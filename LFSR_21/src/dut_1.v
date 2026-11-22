`timescale 1ns/1ns

module galois_lfsr_8b (
    input clk,
    input arst_n,
    output reg [7:0] q
);

    wire sync_rstn;

    // 2-stage reset synchronizer
    synchronizer u0 (
        .clk(clk),
        .qin(arst_n),
        .qout(sync_rstn)
    );

    // Galois LFSR operation
    always @(posedge clk)
    begin
        if (!sync_rstn)
            q <= 8'b00000001;

        else
        begin
            q[7] <= q[6];
            q[6] <= q[5];
            q[5] <= q[4];
            q[4] <= q[3] ^ q[7];
            q[3] <= q[2] ^ q[7];
            q[2] <= q[1] ^ q[7];
            q[1] <= q[0];
            q[0] <= q[7];
        end
    end

endmodule


// 2-stage reset synchronizer
module synchronizer (
    input clk,
    input qin,
    output qout
);

    reg temp_reg0;
    reg temp_reg1;

    always @(posedge clk or negedge qin)
    begin
        if (!qin)
        begin
            temp_reg0 <= 1'b0;
            temp_reg1 <= 1'b0;
        end

        else
        begin
            temp_reg0 <= 1'b1;
            temp_reg1 <= temp_reg0;
        end
    end

    assign qout = temp_reg1;

endmodule

