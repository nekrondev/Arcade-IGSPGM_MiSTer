// ICS2115 Lookup Tables
// Volume (4096×16) and pan law (256×12)
//
// Volume table: registered (1-cycle latency) for timing closure
// Pan table: combinational
// µ-law decode is implemented as a local function in ics2115_osc.

module ics2115_tables (
    input  logic        clk,

    // Volume table — 4096 entries, 16-bit unsigned
    // Input:  12-bit index from (vol.acc >> 14) & 0xFFF
    // Output: 15-bit linear amplitude (registered, 1-cycle latency)
    input  logic [11:0] vol_addr,
    output logic [15:0] vol_data,

    // Pan law — 256 entries, 12-bit
    // Input:  8-bit pan index
    // Output: attenuation value subtracted from volume index
    input  logic [7:0]  pan_addr,
    output logic [11:0] pan_data
);

    // =========================================================================
    // Volume table — hardware-measured integer formula
    // exp  = i[11:8]
    // mant = i[7:0]
    // exp == 0: mant >> 7
    // exp >  0: ceil(((0x100 | mant) << exp) / 512)
    // =========================================================================
    logic [15:0] vol_mem [0:4095];

    initial begin
        for (int i = 0; i < 4096; i++) begin
            if ((i >> 8) == 0)
                vol_mem[i] = (i & 8'hff) >> 7;
            else
                vol_mem[i] = ((((16'h100 | (i & 8'hff)) << ((i >> 8) - 1)) + 16'hff) >> 8);
        end
    end

    // Registered output — 1 cycle latency from address input
    always_ff @(posedge clk) begin
        vol_data <= vol_mem[vol_addr];
    end

    // =========================================================================
    // Pan law table — hardware-measured 16-step attenuation table.
    // Indexed by pan_addr[7:4]. Entry 0 is full attenuation; 12'hfff is
    // equivalent to the measured 4096 for the 12-bit volume index range because
    // the post-pan index is clamped to zero when <= 0.
    // =========================================================================
    always_comb begin
        case (pan_addr[7:4])
            4'h0: pan_data = 12'hfff;
            4'h1: pan_data = 12'd508;
            4'h2: pan_data = 12'd364;
            4'h3: pan_data = 12'd304;
            4'h4: pan_data = 12'd248;
            4'h5: pan_data = 12'd200;
            4'h6: pan_data = 12'd168;
            4'h7: pan_data = 12'd140;
            4'h8: pan_data = 12'd116;
            4'h9: pan_data = 12'd96;
            4'ha: pan_data = 12'd76;
            4'hb: pan_data = 12'd56;
            4'hc: pan_data = 12'd40;
            4'hd: pan_data = 12'd28;
            4'he: pan_data = 12'd12;
            4'hf: pan_data = 12'd0;
        endcase
    end

endmodule
