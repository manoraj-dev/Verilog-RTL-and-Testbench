`timescale 1ns/1ns


module interrupt_controller #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 8,
    parameter NUM_INTR   = 16,
    parameter INTR_SERV = 4
)(
    // APB interface
    input                       pclk_i,
    input                       preset_i,
    input  [ADDR_WIDTH-1:0]     paddr_i,
    input  [DATA_WIDTH-1:0]     pwdata_i,
    output reg [DATA_WIDTH-1:0] prdata_o,
    input                       pwrite_i,
    input                       penable_i,
    input                       psel_i,
    output reg                   pready_o,
    output reg                   perror_o,

    // Interrupt interface
    output reg [INTR_SERV-1:0]   intr_to_service_o,
    input                        intr_serviced_i,
    input  [NUM_INTR-1:0]       intr_active_i,
    output reg                   intr_valid_o
);

    // FSM states
    localparam [3:0] S_NO_INTR = 4'b1000;
    localparam [3:0] S_INTR    = 4'b0100;
    localparam [3:0] S_WAIT    = 4'b0010;
    localparam [3:0] S_ERROR   = 4'b0001;

    reg [3:0] state, next_state;

    // Interrupt priority registers
    reg [INTR_SERV-1:0] priority_reg [0:NUM_INTR-1];

    reg [INTR_SERV-1:0] current_highest_priority;
    reg [INTR_SERV-1:0] interrupt_number;

    integer i;

    // Temporary variables for priority selection
    reg found_interrupt;
    reg [INTR_SERV-1:0] selected_priority;
    reg [INTR_SERV-1:0] selected_interrupt;

    // =====================================================
    // 1. NEXT STATE LOGIC (COMBINATIONAL)
    // =====================================================
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
                next_state = S_WAIT;
            end

            S_WAIT: begin
                if (intr_serviced_i) begin
                    if (intr_active_i != 0)
                        next_state = S_INTR;
                    else
                        next_state = S_NO_INTR;
                end
                else begin
                    next_state = S_WAIT;
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

    // =====================================================
    // 2. SEQUENTIAL LOGIC
    // State, APB registers, and interrupt outputs
    // are assigned ONLY in this block.
    // =====================================================
    always @(posedge pclk_i or posedge preset_i) begin

        if (preset_i) begin

            state                   <= S_NO_INTR;
            pready_o                <= 1'b0;
            perror_o                 <= 1'b0;
            prdata_o                 <= {DATA_WIDTH{1'b0}};
            intr_to_service_o        <= {INTR_SERV{1'b0}};
            intr_valid_o             <= 1'b0;

            current_highest_priority <= {INTR_SERV{1'b0}};
            interrupt_number         <= {INTR_SERV{1'b0}};

            for (i = 0; i < NUM_INTR; i = i + 1)
                priority_reg[i] <= {INTR_SERV{1'b0}};

        end

        else begin

            // Update FSM state
            state <= next_state;

            // Default APB response signals
            pready_o <= 1'b0;
            perror_o  <= 1'b0;
            prdata_o  <= {DATA_WIDTH{1'b0}};

            // -------------------------------------------------
            // APB REGISTER ACCESS
            // -------------------------------------------------
            if (psel_i && penable_i) begin

                pready_o <= 1'b1;

                if (paddr_i < NUM_INTR) begin

                    if (pwrite_i) begin
                        // Write interrupt priority
                        priority_reg[paddr_i] <= pwdata_i;
                    end

                    else begin
                        // Read interrupt priority
                        prdata_o <= priority_reg[paddr_i];
                    end

                end

                else begin
                    // Invalid register address
                    perror_o <= 1'b1;
                end

            end

            // -------------------------------------------------
            // INTERRUPT HANDLING
            // -------------------------------------------------
            case (state)

                S_NO_INTR: begin
                    intr_valid_o <= 1'b0;
                    intr_to_service_o <= {INTR_SERV{1'b0}};
                end

                S_INTR: begin

                    // Initialize priority search
                    selected_priority   = {INTR_SERV{1'b0}};
                    selected_interrupt = {INTR_SERV{1'b0}};
                    found_interrupt   = 1'b0;

                    // Find highest priority active interrupt
                    for (i = 0; i < NUM_INTR; i = i + 1) begin

                        if (intr_active_i[i]) begin

                            if (!found_interrupt ||
                                (priority_reg[i] > selected_priority)) begin

                                selected_priority   = priority_reg[i];
                                selected_interrupt = i;
                                found_interrupt    = 1'b1;

                            end

                        end

                    end

                    current_highest_priority <= selected_priority;
                    interrupt_number         <= selected_interrupt;

                    intr_to_service_o <= selected_interrupt;
                    intr_valid_o      <= found_interrupt;

                end

                S_WAIT: begin

                    // Keep interrupt information until serviced
                    if (intr_serviced_i) begin
                        intr_valid_o      <= 1'b0;
                        intr_to_service_o <= {INTR_SERV{1'b0}};
                    end

                end

                S_ERROR: begin
                    intr_valid_o      <= 1'b0;
                    intr_to_service_o <= {INTR_SERV{1'b0}};
                    perror_o          <= 1'b1;
                end

                default: begin
                    intr_valid_o      <= 1'b0;
                    intr_to_service_o <= {INTR_SERV{1'b0}};
                    state             <= S_NO_INTR;
                end

            endcase

        end

    end

endmodule

