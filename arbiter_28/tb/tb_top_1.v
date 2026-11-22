
`include "../src/dut_1.v"

module tb_top_1;

    reg  [3:0] req;
    wire [3:0] grant;
    wire  irq_valid;
    wire [1:0] irq_id;

arbiter dut(.req(req),.grant(grant),.irq_valid(irq_valid),.irq_id(irq_id) );

    
  task check_result;
        input [3:0] request_value;
        input [3:0] expected_grant;
        input  expected_valid;
        input [1:0] expected_id;

        begin
            req = request_value;
            #10;

            $display("REQ=%b | GRANT=%b | VALID=%b | ID=%b", req, grant, irq_valid, irq_id);

            if ((grant !== expected_grant) ||
                (irq_valid !== expected_valid) ||
                (irq_id !== expected_id)) begin

                $display("FAIL: Expected GRANT=%b VALID=%b ID=%b",expected_grant, expected_valid, expected_id);
            end

            else begin
                $display("PASS");
            end

            $display("---------------------------------------");
        end
    endtask

    initial begin

        $dumpfile("waveform.vcd");
        $dumpvars(0, tb_top_1);

        req = 4'b0000;

        $display(" Arbiter Test Started");
        $display("---------------------------------------");

        
        check_result(4'b0000, 4'b0000, 1'b0, 2'b00);

       
        check_result(4'b0001, 4'b0001, 1'b1, 2'b00);
        check_result(4'b0010, 4'b0010, 1'b1, 2'b01);
        check_result(4'b0100, 4'b0100, 1'b1, 2'b10);
        check_result(4'b1000, 4'b1000, 1'b1, 2'b11);

    
        check_result(4'b1010, 4'b0010, 1'b1, 2'b01);
        check_result(4'b1100, 4'b0100, 1'b1, 2'b10);
        check_result(4'b1110, 4'b0010, 1'b1, 2'b01);
        check_result(4'b1111, 4'b0001, 1'b1, 2'b00);

        $display(" Arbiter Test Completed");

        #10;
        $finish;

    end

endmodule

