//============================================================================
//  Copyright (C) 2026 Martin Donlon
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
//============================================================================

module coin_pulse(
    input clk,

    input vblank,

    input button,
    output pulse
);

reg [15:0] shift;
reg prev_button;
reg prev_vblank;

assign pulse = |shift[3:0];

always_ff @(posedge clk) begin
    prev_vblank <= vblank;
    prev_button <= button;

    if (vblank & ~prev_vblank) begin
        shift <= { shift[14:0], 1'b0 };
    end

    if (~|shift & ~button & prev_button) begin
        shift[0] <= 1;
    end
end

endmodule
