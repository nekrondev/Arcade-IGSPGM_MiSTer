package gba.cpu

import chisel3._
import chisel3.util._

class Multiplier extends Module {
  val io = IO(new Bundle {
    val enable = Input(Bool())

    val a = Input(UInt(32.W))
    val b = Input(UInt(32.W))

    val start = Input(Bool())
    val loadAccumulator = Input(Bool())
    val accumulate = Input(Bool())
    val signed = Input(Bool())
    val long = Input(Bool())

    val outLo = Output(UInt(32.W))
    val outHi = Output(UInt(32.W))
    val done = Output(Bool())
    val outFlagZ = Output(Bool())
    val outFlagN = Output(Bool())

    /// Save/restore state port. Local words:
    ///   0: accumulator[31:0]  1: accumulator[63:32]
    ///   2: output[31:0]       3: output[63:32]
    ///   4: counter
    val state = new StatePort
  })

  val accumulator = Reg(UInt(64.W))
  val output = Reg(UInt(64.W))
  val counter = Reg(UInt(2.W))
  // Timing-closure split (see the start block below). For m >= 2 the full 32x32
  // product is captured here and the 64-bit accumulate add is deferred one cycle
  // into the countdown the FSM already spends, so no instruction cycle is added.
  // `addPending` marks that deferred add. Both are dead at instruction boundaries
  // (freeze only engages once `done`), so neither needs a savestate word.
  val product = Reg(UInt(64.W))
  val addPending = RegInit(false.B)

  // Determine cycle length.
  // Early termination based on the number of leading 0s or 1s.
  // For unsigned long multiply, it's only leading 0s.
  val prefixZeroes = VecInit(!io.b(31, 24).orR, !io.b(23, 16).orR, !io.b(15, 8).orR, !io.b(7, 0).orR).asUInt
  val prefixOnes = VecInit(io.b(31, 24).andR, io.b(23, 16).andR, io.b(15, 8).andR, io.b(7, 0).andR).asUInt
  val termOnes = !(io.long && !io.signed)
  val numCycles = MuxCase(3.U, Seq(
    (prefixZeroes(2, 0) === "b111".U || (prefixOnes(2, 0) === "b111".U && termOnes)) -> 0.U,
    (prefixZeroes(1, 0) === "b11".U || (prefixOnes(1, 0) === "b11".U && termOnes)) -> 1.U,
    (prefixZeroes(0, 0) === "b1".U || (prefixOnes(0, 0) === "b1".U && termOnes)) -> 2.U,
  ))

  val augend = Mux(io.accumulate, accumulator, 0.U)

  // Full 32x32 product (signedness as today). Used for m >= 2, where the 64-bit
  // accumulate add is split off into the next cycle so this stage is multiply-only.
  val fullProduct = Mux(io.signed, (io.a.asSInt * io.b.asSInt).asUInt, io.a * io.b)

  // Small 32x9 product for m == 1 (the only case with no spare cycle: the result
  // is read one cycle after start). When m == 1 the multiplier operand Rs fits in
  // 8 bits of magnitude, so bits[31:8] are all equal to bit 8 and the full product
  // equals a * b[8:0]. A 32x9 multiply + the 64-bit add closes in a single cycle.
  //   Signedness: signed long uses signed; unsigned long uses unsigned (and m == 1
  //   there only via leading zeroes, so b[8] == 0); non-long (MUL/MLA) only needs
  //   the low 32 bits, which a signed 32x9 reproduces for both prefix cases — so
  //   force signed unless this is an unsigned *long* multiply.
  val b9 = io.b(8, 0)
  val smallSigned = io.signed || !io.long
  val smallProductS = Wire(SInt(64.W)); smallProductS := io.a.asSInt * b9.asSInt
  val smallProductU = Wire(UInt(64.W)); smallProductU := io.a * b9
  val smallProduct = Mux(smallSigned, smallProductS.asUInt, smallProductU)

  when (io.enable) {
    when (io.loadAccumulator) {
      accumulator := Cat(io.a, io.b)
    }
    when (io.start) {
      when (numCycles === 0.U) {
        // m == 1: small multiply + accumulate, all in this cycle.
        output := smallProduct + augend
        addPending := false.B
      } .otherwise {
        // m >= 2: capture the full product now, add the augend next cycle.
        product := fullProduct
        addPending := true.B
      }
      counter := numCycles
    } .elsewhen (addPending) {
      // Deferred accumulate add — lands on the first countdown cycle, which the
      // FSM already spends before `done`, so no instruction cycle is added.
      output := product + augend
      addPending := false.B
    }
    when (counter > 0.U) {
      counter := counter - 1.U
    }
  }

  // Save/restore: 64-bit registers are read/written 32 bits at a time via
  // read-modify-write. Placed after the enable block so it wins while frozen.
  io.state.readData := MuxLookup(io.state.address, 0.U)(Seq(
    0.U -> accumulator(31, 0),
    1.U -> accumulator(63, 32),
    2.U -> output(31, 0),
    3.U -> output(63, 32),
    4.U -> counter,
  ))
  when (io.state.writeEnable) {
    switch (io.state.address) {
      is (0.U) { accumulator := Cat(accumulator(63, 32), io.state.writeData) }
      is (1.U) { accumulator := Cat(io.state.writeData, accumulator(31, 0)) }
      is (2.U) { output := Cat(output(63, 32), io.state.writeData) }
      is (3.U) { output := Cat(io.state.writeData, output(31, 0)) }
      is (4.U) { counter := io.state.writeData(1, 0) }
    }
  }

  val zeroLo = output(31, 0) === 0.U
  val zeroHi = output(63, 32) === 0.U

  io.done := counter === 0.U
  io.outLo := output(31, 0)
  io.outHi := output(63, 32)
  io.outFlagZ := Mux(io.long, zeroLo && zeroHi, zeroLo)
  io.outFlagN := Mux(io.long, output(63), output(31))
}
