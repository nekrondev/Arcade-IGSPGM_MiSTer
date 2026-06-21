package gba.cpu

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import gba.mem.{BusAccessWidth, BusInterface}
import lib.log.Logger

/// ARM7TDMI-S compatible processor as found in the GBA
class ARM7TDMI extends Module {
  val io = IO(new Bundle {
    /// Global enable signal for emulation
    val enable = Input(Bool())
    /// Debug output
    val debug = Output(new CpuDebug())

    /// Memory bus interface
    val mem = new BusInterface
    /// **Active-High** fast interrupt request
    val FIQ = Input(Bool())
    /// **Active-High** interrupt request
    val IRQ = Input(Bool())

    /// Save/restore: indexed 32-bit access to the full machine state (56 words).
    val state = new StatePort
    /// Request the CPU to halt at the next safe (instruction-boundary) point.
    val saveReq = Input(Bool())
    /// High (and held) once the CPU is frozen at a safe point for save/restore.
    val safe = Output(Bool())
  })
  val logger = Logger("cpu", enable = io.enable)

  // Save/restore handshake: when saveReq is asserted, run until the next
  // instruction boundary (with the multiplier idle), then freeze by gating
  // `enable` low and hold `safe`. While frozen, all `when(enable)` updates stop;
  // the host reads/writes state over `io.state`. Deasserting saveReq resumes from
  // exactly the frozen cycle. The host must also snapshot external memory so the
  // pending instruction prefetch re-presents identically on resume.
  // `freeze` is driven below, once controlUnit and multiplier are instantiated.
  val freeze = Wire(Bool())
  val enable = io.enable && io.mem.CLKEN && !freeze

  ////////////////////////////////// Busses and Registers //////////////////////////////////
  val memAddrReg = Reg(UInt(32.W))
  val memReadDataReg = Reg(UInt(32.W))

  val aBus = Wire(UInt(32.W))
  val bBus = Wire(UInt(32.W))
  val cBus = Wire(UInt(32.W))
  val pcBus = Wire(UInt(32.W))
  val aluBus = Wire(UInt(32.W))
  val aluConditionOut = Wire(new ConditionFlags)
  val incrementerBus = Wire(UInt(32.W))
  val control = Wire(new ControlSignals)
  val cpsrBus = Wire(new ProgramStatusRegister)
  bBus := DontCare

  //////////////////////////////// Instruction Fetch & Decode //////////////////////////////
  val decodeUnit = Module(new Decoder)
  decodeUnit.io.enable := enable
  decodeUnit.io.advancePipeline := control.advancePipeline
  decodeUnit.io.flushPipeline := control.flushPipeline
  decodeUnit.io.readData := io.mem.RDATA
  decodeUnit.io.readAddress := memAddrReg
  decodeUnit.io.thumb := cpsrBus.thumb

  ////////////////////////////////////// Control Unit //////////////////////////////////////
  val controlUnit = Module(new Control)
  controlUnit.io.enable := enable
  controlUnit.io.nextInstruction := decodeUnit.io.decoded
  controlUnit.io.fiq := io.FIQ
  controlUnit.io.irq := io.IRQ
  control := controlUnit.io.signals
  when (control.busB === BusBValue.Immediate) {
    bBus := control.immediate
  }

  ///////////////////////////////////// Register File //////////////////////////////////////
  // Working-set register file: `active` always holds the current mode's r0-r15, so the
  // hot-path operand reads and the writeback hit it directly -- no banking mux, no offset
  // adder, and half the writeback fan-out of the old flat 31-entry file. The inactive
  // banked copies live in `bank*` and are swapped with `active` only on a mode change
  // (see the swap block in `when(enable)` below); mode changes are rare and the swap is
  // atomic with the cpsr.mode update, so it costs no extra cycle.
  //   bank_r13/bank_r14: indexed by mode group 0=User/System 1=Fiq 2=Irq 3=Supervisor
  //                      4=Abort 5=Undefined (the active group's slot is stale).
  //   bank_r8_12: the *inactive* r8-r12 group (FIQ's copy when not in FIQ; User's in FIQ).
  val active     = RegInit(VecInit.fill(16)(0.U(32.W)))
  val bank_r13   = RegInit(VecInit.fill(6)(0.U(32.W)))
  val bank_r14   = RegInit(VecInit.fill(6)(0.U(32.W)))
  val bank_r8_12 = RegInit(VecInit.fill(5)(0.U(32.W)))

  private def modeToGroup(mode: CpuMode.Type): UInt = MuxLookup(mode, 0.U)(Seq(
    CpuMode.Fiq        -> 1.U,
    CpuMode.Irq        -> 2.U,
    CpuMode.Supervisor -> 3.U,
    CpuMode.Abort      -> 4.U,
    CpuMode.Undefined  -> 5.U,
  )) // User / System -> group 0

  // User-bank access for LDM/STM with `^` (regUserReadC / regUserWrite): read/write the
  // User-mode view regardless of current mode. Off the hot path. Only FIQ banks r8-r12,
  // and only non-{User,System} modes bank r13/r14 away from the User copies.
  private def userBankIdx8_12(index: UInt): UInt =
    Mux((index >= 8.U) && (index <= 12.U), index - 8.U, 0.U)
  private def userRegRead(index: UInt): UInt = {
    val inFiq  = cpsr.mode === CpuMode.Fiq
    val inGrp0 = (cpsr.mode === CpuMode.User) || (cpsr.mode === CpuMode.System)
    Mux(index === 13.U, Mux(inGrp0, active(13.U), bank_r13(0.U)),
    Mux(index === 14.U, Mux(inGrp0, active(14.U), bank_r14(0.U)),
    Mux((index >= 8.U) && (index <= 12.U),
        Mux(inFiq, bank_r8_12(userBankIdx8_12(index)), active(index)),
        active(index))))                                            // r0-r7, r15
  }
  private def userRegWrite(index: UInt, data: UInt): Unit = {
    val inFiq  = cpsr.mode === CpuMode.Fiq
    val inGrp0 = (cpsr.mode === CpuMode.User) || (cpsr.mode === CpuMode.System)
    when (index === 13.U) {
      when (inGrp0) { active(13.U) := data } .otherwise { bank_r13(0.U) := data }
    } .elsewhen (index === 14.U) {
      when (inGrp0) { active(14.U) := data } .otherwise { bank_r14(0.U) := data }
    } .elsewhen ((index >= 8.U) && (index <= 12.U)) {
      when (inFiq) { bank_r8_12(userBankIdx8_12(index)) := data } .otherwise { active(index) := data }
    } .otherwise {
      active(index) := data                                        // r0-r7, r15
    }
  }

  val cpsr = RegInit((new ProgramStatusRegister).Lit(
    _.mode -> CpuMode.System,
    _.thumb -> false.B,
    _.irqDisable -> true.B,
    _.fiqDisable -> true.B,
    _.padding -> 0.U,
    _.cond -> (new ConditionFlags).Lit(
      _.n -> false.B,
      _.z -> false.B,
      _.c -> false.B,
      _.v -> false.B,
    ),
  ))
  val spsrVec = Reg(Vec(5, new ProgramStatusRegister))
  val spsrIndex = MuxLookup(control.regBankMode, 0.U)(Seq(
    CpuMode.Supervisor -> 0.U,
    CpuMode.Abort -> 1.U,
    CpuMode.Undefined -> 2.U,
    CpuMode.Irq -> 3.U,
    CpuMode.Fiq -> 4.U,
  ))
  val spsr = spsrVec(spsrIndex)
  val modeHasSpsr = (control.regBankMode =/= CpuMode.User) && (control.regBankMode =/= CpuMode.System)
  val modePrivileged = cpsr.mode =/= CpuMode.User
  val nextCpsr = WireDefault(cpsr)

  controlUnit.io.currentStatus := cpsr
  controlUnit.io.nextStatus := nextCpsr
  cpsrBus := cpsr
  val pc = active(15.U)
  pcBus := pc
  // `active` already holds the current mode's r0-r15, so operand reads are a direct
  // 16:1 read -- no banking mux, no offset adder (the old 50 MHz critical front).
  aBus := active(control.regReadA)
  when (control.busB === BusBValue.RegisterB) {
    bBus := active(control.regReadB)
  } .elsewhen (control.busB === BusBValue.Cpsr) {
    bBus := cpsr.asUInt
  } .elsewhen (control.busB === BusBValue.Spsr) {
    when (modeHasSpsr) {
      bBus := spsr.asUInt
    } .otherwise {
      // Modes without SPSR apparently return CPSR on a read
      bBus := cpsr.asUInt
    }
  }
  // cBus: current mode normally; the User bank for LDM/STM with `^` (off the hot path).
  cBus := Mux(control.regUserReadC, userRegRead(control.regReadC), active(control.regReadC))
  when (enable) {
    when (control.cpsrUpdateCond) {
      nextCpsr.cond := aluConditionOut
    }
    when (control.cpsrUpdateThumb) {
      nextCpsr.thumb := aBus(0)
    }
    when (control.cpsrUpdateFields(0) && modePrivileged) {
      nextCpsr.mode := suppressEnumCastWarning { aluBus(4, 0).asTypeOf(CpuMode()) }
      nextCpsr.thumb := aluBus(5)
      nextCpsr.fiqDisable := aluBus(6)
      nextCpsr.irqDisable := aluBus(7)
    }
    when (control.cpsrUpdateFields(1)) {
      nextCpsr.cond := aluBus(31, 28).asTypeOf(new ConditionFlags)
    }
    when (control.spsrUpdateFields(0) && modeHasSpsr) {
      spsr.mode := suppressEnumCastWarning { aluBus(4, 0).asTypeOf(CpuMode()) }
      spsr.thumb := aluBus(5)
      spsr.fiqDisable := aluBus(6)
      spsr.irqDisable := aluBus(7)
    }
    when (control.spsrUpdateFields(1) && modeHasSpsr) {
      spsr.cond := aluBus(31, 28).asTypeOf(new ConditionFlags)
    }
    when (control.cpsrRestore && modeHasSpsr) {
      nextCpsr := spsr
    }
    when (control.startException) {
      val newMode = control.regBankMode
      nextCpsr.mode := newMode
      nextCpsr.thumb := false.B
      nextCpsr.irqDisable := true.B
      when (newMode === CpuMode.Fiq) { // also in Reset
        nextCpsr.fiqDisable := true.B
      }
      spsr := cpsrBus
    }

    // Mode-change bank swap: one detection point for every mode-change site above
    // (exception entry, MSR, exception return). The swap is just concurrent registered
    // moves done at the same edge as the cpsr.mode change, so it adds no cycle. r13/r14
    // bank across mode groups; r8-r12 only across the FIQ boundary. (User<->System are
    // the same group, so they correctly do nothing.)
    val modeChanging = nextCpsr.mode =/= cpsr.mode
    when (modeChanging) {
      val oldGrp = modeToGroup(cpsr.mode)
      val newGrp = modeToGroup(nextCpsr.mode)
      when (oldGrp =/= newGrp) {
        bank_r13(oldGrp) := active(13.U)              // save outgoing mode's r13/r14
        bank_r14(oldGrp) := active(14.U)
        active(13.U) := bank_r13(newGrp)              // load incoming mode's r13/r14
        active(14.U) := bank_r14(newGrp)
      }
      when ((cpsr.mode === CpuMode.Fiq) =/= (nextCpsr.mode === CpuMode.Fiq)) {
        for (i <- 0 until 5) {                        // swap r8-r12 across the FIQ boundary
          active((i + 8).U) := bank_r8_12(i.U)
          bank_r8_12(i.U)   := active((i + 8).U)
        }
      }
    }

    // Register writeback -- after the swap, so an explicit write (e.g. exception entry's
    // LR_newmode := PC) wins over the swap-load for that register. `active` is the current
    // mode's view, so a normal write is a direct 16-target write (half the old fan-out);
    // the User bank (LDM/STM `^`) takes the off-hot-path helper.
    when (control.regWriteEnable) {
      logger.debug(cf"  reg write [${control.regWriteIndex}] <- ${aluBus}%x")
      when (control.regUserWrite) {
        userRegWrite(control.regWriteIndex, aluBus)
      } .otherwise {
        active(control.regWriteIndex) := aluBus
      }
    }

    switch (control.pcNext) {
      is (PcNext.Incrementer) {
        // PC is always aligned to 2
        //  THUMB: part of the ARM ARM
        //    ARM: "unpredictable", but seems to be the same behavior
        pc := incrementerBus & "hFFFFFFFE".U(32.W)
      }
    }
    cpsr := nextCpsr
  }

  ///////////////////////////////////// Barrel Shifter /////////////////////////////////////
  val shifter = Module(new Shifter)
  shifter.io.in := bBus
  shifter.io.carryIn := cpsrBus.cond.c
  shifter.io.shiftKind := control.shiftKind
  shifter.io.shiftAmount := control.shiftImmediate
  shifter.io.latchShift := enable && control.shiftDoLatch
  shifter.io.useLatchedShift := control.shiftUseLatched

  ////////////////////////////////////////// ALU ///////////////////////////////////////////
  val alu = Module(new Alu)
  alu.io.a := aBus
  when (control.aluInAAlign4) {
    alu.io.a := aBus & "hFFFFFFFC".U(32.W)
  }
  alu.io.b := shifter.io.out
  alu.io.opcode := control.aluOpcode
  alu.io.flagIn := cpsrBus.cond
  alu.io.shifterCarry := shifter.io.carryOut
  aluBus := alu.io.out
  when (control.aluOutAlign4) {
    aluBus := alu.io.out & "hFFFFFFFC".U(32.W)
  }
  aluConditionOut := alu.io.flagOut

  /////////////////////////////////////// Multiplier ///////////////////////////////////////
  val multiplier = Module(new Multiplier)
  multiplier.io.enable := enable
  multiplier.io.a := aBus
  multiplier.io.b := bBus
  multiplier.io.start := control.multiplyEnable
  multiplier.io.loadAccumulator := control.multiplyLoadAccumulator
  multiplier.io.accumulate := control.multiplyAccumulate
  multiplier.io.signed := control.multiplySigned
  multiplier.io.long := control.multiplyLong
  when (control.busB === BusBValue.MultiplyLo) {
    bBus := multiplier.io.outLo
  } .elsewhen (control.busB === BusBValue.MultiplyHi) {
    bBus := multiplier.io.outHi
  }
  controlUnit.io.multiplierDone := multiplier.io.done
  when (control.cpsrFromMultiply) {
    nextCpsr.cond.z := multiplier.io.outFlagZ
    nextCpsr.cond.n := multiplier.io.outFlagN
  }

  /////////////////////////////////////// Incrementer //////////////////////////////////////
  incrementerBus := memAddrReg + Mux(cpsrBus.thumb && !control.incrementerForceWord, 2.U, 4.U)

  ///////////////////////////////////////// IO Port ////////////////////////////////////////
  val currentMemReadWidth = Reg(BusAccessWidth())
  val lastMemReadWidth = Reg(BusAccessWidth())
  val lastMemReadAlign = Reg(UInt(2.W))
  io.mem.ADDR := memAddrReg
  switch (control.addressSource) {
    is (AddressSource.Incrementer) { io.mem.ADDR := incrementerBus }
    is (AddressSource.Pc) { io.mem.ADDR := pcBus }
    is (AddressSource.Alu) { io.mem.ADDR := aluBus }
    is (AddressSource.Immediate) { io.mem.ADDR := control.immediate }
  }
  when (enable) {
    memAddrReg := io.mem.ADDR
    currentMemReadWidth := io.mem.SIZE
    when (control.latchMemReadData) {
      lastMemReadWidth := currentMemReadWidth
      lastMemReadAlign := memAddrReg(1, 0)
      memReadDataReg := io.mem.RDATA
    }
  }
  val memWriteData = Wire(UInt(32.W))
  when (currentMemReadWidth === BusAccessWidth.Byte) {
    memWriteData := Fill(4, cBus(7, 0))
  } .elsewhen (currentMemReadWidth === BusAccessWidth.Halfword) {
    memWriteData := Fill(2, cBus(15, 0))
  } .otherwise {
    memWriteData := cBus
  }
  when (control.busB === BusBValue.MemReadData) {
    val readData = WireDefault(memReadDataReg)
    bBus := readData

    // For halfword and byte loads, mask out / sign extend bits.
    val maskValue = WireDefault(0.U(8.W))
    when (control.memReadDataSigned) {
      val signByte = Mux(
        lastMemReadWidth === BusAccessWidth.Halfword,
        lastMemReadAlign | 1.U,
        lastMemReadAlign,
      )
      maskValue := Fill(8, memReadDataReg(Cat(signByte, "b111".U(3.W))))
    }
    when (lastMemReadWidth === BusAccessWidth.Byte) {
      readData := Cat(
        Mux(lastMemReadAlign === 3.U, memReadDataReg(31, 24), maskValue),
        Mux(lastMemReadAlign === 2.U, memReadDataReg(23, 16), maskValue),
        Mux(lastMemReadAlign === 1.U, memReadDataReg(15, 8), maskValue),
        Mux(lastMemReadAlign === 0.U, memReadDataReg(7, 0), maskValue),
      )
    } .elsewhen (lastMemReadWidth === BusAccessWidth.Halfword) {
      readData := Cat(
        Mux(lastMemReadAlign(1), memReadDataReg(31, 16), Fill(2, maskValue)),
        Mux(!lastMemReadAlign(1), memReadDataReg(15, 0), Fill(2, maskValue)),
      )
    }
  }
  when (control.shiftByAddressAlign) {
    shifter.io.shiftAmount := lastMemReadAlign << 3
  }

  io.mem.WDATA := memWriteData
  io.mem.WRITE := control.memWrite
  io.mem.SIZE := control.memWidth
  io.mem.MREQ := control.memRequest
  io.mem.SEQ := control.memSequential
  io.mem.LOCK := control.memLock
  io.mem.PROT := control.memProt

  ///////////////////////////////////// Save / Restore /////////////////////////////////////
  // Drive the handshake now that controlUnit and multiplier exist. Once a safe
  // point is reached it is *latched* (`halted`): subsequent state writes during a
  // restore change control state (and thus `atSafePoint`), but the CPU must stay
  // frozen until the host releases `saveReq`.
  val atSafePoint = controlUnit.io.atBoundary && multiplier.io.done
  val halted = RegInit(false.B)
  when (io.saveReq && atSafePoint) { halted := true.B }
  when (!io.saveReq) { halted := false.B }
  freeze := io.saveReq && (halted || atSafePoint)
  io.safe := freeze

  // Route the shared 32-bit state port. Global word map (NOTE: layout changed with the
  // working-set register file -- savestate format is NOT compatible with older builds):
  //   0..15   active[0..15] (current mode r0-r15)
  //   16..21  bank_r13[0..5]   22..27 bank_r14[0..5]   28..32 bank_r8_12[0..4]
  //   33 cpsr, 34..38 spsr, 39 memAddrReg, 40 memReadDataReg,
  //   41 {lastMemReadWidth, currentMemReadWidth, lastMemReadAlign},
  //   42..46 Decoder, 47..51 Control, 52..56 Multiplier, 57 Shifter.
  for ((unit, base, end) <- Seq(
    (decodeUnit.io.state, 42, 47), (controlUnit.io.state, 47, 52),
    (multiplier.io.state, 52, 57), (shifter.io.state, 57, 58),
  )) {
    unit.address := io.state.address - base.U
    unit.writeData := io.state.writeData
    unit.writeEnable := io.state.writeEnable && io.state.address >= base.U && io.state.address < end.U
  }

  // Read mux over the whole map.
  io.state.readData := 0.U
  when (io.state.address < 16.U) {
    io.state.readData := active(io.state.address)
  } .elsewhen (io.state.address < 22.U) {
    io.state.readData := bank_r13(io.state.address - 16.U)
  } .elsewhen (io.state.address < 28.U) {
    io.state.readData := bank_r14(io.state.address - 22.U)
  } .elsewhen (io.state.address < 33.U) {
    io.state.readData := bank_r8_12(io.state.address - 28.U)
  } .elsewhen (io.state.address === 33.U) {
    io.state.readData := cpsr.asUInt
  } .elsewhen (io.state.address < 39.U) {
    io.state.readData := spsrVec(io.state.address - 34.U).asUInt
  } .elsewhen (io.state.address === 39.U) {
    io.state.readData := memAddrReg
  } .elsewhen (io.state.address === 40.U) {
    io.state.readData := memReadDataReg
  } .elsewhen (io.state.address === 41.U) {
    io.state.readData := Cat(lastMemReadWidth.asUInt.pad(2), currentMemReadWidth.asUInt.pad(2), lastMemReadAlign)
  } .elsewhen (io.state.address < 47.U) {
    io.state.readData := decodeUnit.io.state.readData
  } .elsewhen (io.state.address < 52.U) {
    io.state.readData := controlUnit.io.state.readData
  } .elsewhen (io.state.address < 57.U) {
    io.state.readData := multiplier.io.state.readData
  } .otherwise {
    io.state.readData := shifter.io.state.readData
  }

  // Write the top-level registers (submodules handle their own words). Gated only
  // by writeEnable (independent of `enable`); placed after every when(enable)
  // block so it wins while the CPU is frozen.
  when (io.state.writeEnable) {
    suppressEnumCastWarning {
      when (io.state.address < 16.U) {
        active(io.state.address) := io.state.writeData
      } .elsewhen (io.state.address < 22.U) {
        bank_r13(io.state.address - 16.U) := io.state.writeData
      } .elsewhen (io.state.address < 28.U) {
        bank_r14(io.state.address - 22.U) := io.state.writeData
      } .elsewhen (io.state.address < 33.U) {
        bank_r8_12(io.state.address - 28.U) := io.state.writeData
      } .elsewhen (io.state.address === 33.U) {
        cpsr := io.state.writeData.asTypeOf(new ProgramStatusRegister)
      } .elsewhen (io.state.address < 39.U) {
        spsrVec(io.state.address - 34.U) := io.state.writeData.asTypeOf(new ProgramStatusRegister)
      } .elsewhen (io.state.address === 39.U) {
        memAddrReg := io.state.writeData
      } .elsewhen (io.state.address === 40.U) {
        memReadDataReg := io.state.writeData
      } .elsewhen (io.state.address === 41.U) {
        lastMemReadAlign := io.state.writeData(1, 0)
        currentMemReadWidth := io.state.writeData(3, 2).asTypeOf(BusAccessWidth())
        lastMemReadWidth := io.state.writeData(5, 4).asTypeOf(BusAccessWidth())
      }
    }
  }

  ////////////////////////////////////////// Debug /////////////////////////////////////////
  io.debug.registers := active   // current mode's r0-r15
  io.debug.cpsr := cpsr.asUInt
  when (enable) {
    logger.debug(cf" r0: ${active(0)}%x   r1: ${active(1)}%x   r2: ${active(2)}%x   r3: ${active(3)}%x")
    logger.debug(cf" r4: ${active(4)}%x   r5: ${active(5)}%x   r6: ${active(6)}%x   r7: ${active(7)}%x")
    logger.debug(cf" r8: ${active(8)}%x   r9: ${active(9)}%x  r10: ${active(10)}%x  r11: ${active(11)}%x")
    logger.debug(cf"r12: ${active(12)}%x  r13: ${active(13)}%x  r14: ${active(14)}%x  r15: ${active(15)}%x")
    logger.debug(cf"cpsr: ${cpsr.asUInt}%x")
  }
}

class ConditionFlags extends Bundle {
  /// Negative or less than
  val n = Bool()
  /// Zero
  val z = Bool()
  /// Carry or borrow or extend
  val c = Bool()
  /// Overflow
  val v = Bool()
}

class ProgramStatusRegister extends Bundle {
  /// [31:28]: Condition flags
  val cond = new ConditionFlags

  /// 20 bits of padding, always read as 0
  val padding = UInt(20.W)

  ///     7: IRQ disable
  val irqDisable = Bool()
  ///     6: FIQ disable
  val fiqDisable = Bool()
  ///     5: State bit
  val thumb = Bool()
  /// [4:0]: Mode bits
  val mode = CpuMode()
}

object CpuMode extends ChiselEnum {
  val User = Value("b10000".U(5.W))
  val Fiq = Value("b10001".U(5.W))
  val Irq = Value("b10010".U(5.W))
  val Supervisor = Value("b10011".U(5.W))
  val Abort = Value("b10111".U(5.W))
  val Undefined = Value("b11011".U(5.W))
  val System = Value("b11111".U(5.W))
}

class CpuDebug extends Bundle {
  val registers = Vec(16, UInt(32.W))
  val cpsr = UInt(32.W)
}