`timescale 1ns/1ns

module interrupt_controller #(
    parameter S_NO_INTR = 4'b1000,
    parameter S_INTR    = 4'b0100,
    parameter S_WAIT    = 4'b0010,
    parameter S_ERROR   = 4'b0001,

    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 8,
    parameter NUM_INTR   = 16,
    parameter INTR_SERV  = 4
)(
    // APB interface
    input pclk_i,
    input preset_i,
    input [ADDR_WIDTH-1:0] paddr_i,
    input [DATA_WIDTH-1:0] pwdata_i,
    output reg [DATA_WIDTH-1:0] prdata_o,
    input pwrite_i,
    input penable_i,
    input psel_i,
    output reg pready_o,
    output reg perror_o,

    // Interrupt interface
    output reg [INTR_SERV-1:0] intr_to_service_o,
    input intr_serviced_i,
    output reg intr_valid_o,
    input [NUM_INTR-1:0] intr_active_i
);

    // State registers
    reg [3:0] state;
    reg [3:0] next_state;

    // Priority registers
    reg [INTR_SERV-1:0] priority_reg [0:NUM_INTR-1];

    // Priority selection variables
    reg [INTR_SERV-1:0] best_priority;
    reg [INTR_SERV-1:0] best_interrupt;

    integer i;
    integer j;

    // ------------------------------------------------
    // 1. APB REGISTER PROGRAMMING LOGIC
    // ------------------------------------------------

    always @(posedge pclk_i) begin

        if (preset_i) begin

            pready_o <= 1'b0;
            perror_o <= 1'b0;
            prdata_o <= {DATA_WIDTH{1'b0}};

            for (i = 0; i < NUM_INTR; i = i + 1)
                priority_reg[i] <= {INTR_SERV{1'b0}};

        end

        else begin

            pready_o <= 1'b0;
            perror_o <= 1'b0;

            if (psel_i && penable_i) begin

                pready_o <= 1'b1;

                if (paddr_i < NUM_INTR) begin

                    if (pwrite_i) begin

                        // Write priority register
                        priority_reg[paddr_i] <= pwdata_i;

                    end

                    else begin

                        // Read priority register
                        prdata_o <= priority_reg[paddr_i];

                    end

                end

                else begin

                    // Invalid APB address
                    perror_o <= 1'b1;
                    prdata_o <= {DATA_WIDTH{1'b0}};

                end

            end
        end
    end


    // ------------------------------------------------
    // 2. HIGHEST PRIORITY INTERRUPT SELECTION
    // ------------------------------------------------

    always @(*) begin

        best_priority = {INTR_SERV{1'b0}};
        best_interrupt = {INTR_SERV{1'b0}};

        for (j = 0; j < NUM_INTR; j = j + 1) begin

            if (intr_active_i[j]) begin

                if (priority_reg[j] > best_priority) begin

                    best_priority = priority_reg[j];
                    best_interrupt = j;

                end
            end
        end
    end


    // ------------------------------------------------
    // 3. FSM NEXT STATE LOGIC
    // ------------------------------------------------

    always @(*) begin

        next_state = state;

        case (state)

            S_NO_INTR: begin

                if (intr_active_i != 0)
                    next_state = S_INTR;

                else
                    next_state = S_NO_INTR;

            end


            S_INTR: begin

                if (intr_active_i != 0)
                    next_state = S_WAIT;

                else
                    next_state = S_NO_INTR;

            end


            S_WAIT: begin

                if (intr_serviced_i) begin

                    if (intr_active_i != 0)
                        next_state = S_INTR;

                    else
                        next_state = S_NO_INTR;

                end

            end


            S_ERROR: begin

                next_state = S_ERROR;

            end


            default: begin

                next_state = S_NO_INTR;

            end

        endcase
    end


    // ------------------------------------------------
    // 4. FSM STATE REGISTER
    // ------------------------------------------------

    always @(posedge pclk_i) begin

        if (preset_i)
            state <= S_NO_INTR;

        else
            state <= next_state;

    end


    // ------------------------------------------------
    // 5. INTERRUPT OUTPUT CONTROL
    // ------------------------------------------------

    always @(posedge pclk_i) begin

        if (preset_i) begin

            intr_to_service_o <= {INTR_SERV{1'b0}};
            intr_valid_o <= 1'b0;

        end

        else begin

            case (state)

                S_NO_INTR: begin

                    intr_to_service_o <= {INTR_SERV{1'b0}};
                    intr_valid_o <= 1'b0;

                end


                S_INTR: begin

                    if (intr_active_i != 0) begin

                        intr_to_service_o <= best_interrupt;
                        intr_valid_o <= 1'b1;

                    end

                    else begin

                        intr_to_service_o <= {INTR_SERV{1'b0}};
                        intr_valid_o <= 1'b0;

                    end

                end


                S_WAIT: begin

                    if (intr_serviced_i) begin

                        intr_to_service_o <= {INTR_SERV{1'b0}};
                        intr_valid_o <= 1'b0;

                    end

                end


                S_ERROR: begin

                    intr_to_service_o <= {INTR_SERV{1'b0}};
                    intr_valid_o <= 1'b0;

                end


                default: begin

                    intr_to_service_o <= {INTR_SERV{1'b0}};
                    intr_valid_o <= 1'b0;

                end

            endcase

        end
    end

endmodule

