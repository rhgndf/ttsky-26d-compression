`default_nettype none

module psram_qpi (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rd_late,
    input  wire       start,
    input  wire       we,
    input  wire [17:0] addr,
    input  wire [7:0] wdata,
    input  wire       more,
    input  wire [3:0] sd_in,
    output reg        psram_sck,
    output reg        psram_cs_n,
    output reg  [3:0] sd_out,
    output reg  [3:0] sd_oe,
    output reg        ready,
    output wire       busy,
    output reg        bdone,
    output reg  [7:0] rdata
);

    localparam [3:0] S_INIT_F5   = 4'd0;
    localparam [3:0] S_INIT_GAP  = 4'd1;
    localparam [3:0] S_INIT_35   = 4'd2;
    localparam [3:0] S_IDLE      = 4'd3;
    localparam [3:0] S_COMMAND   = 4'd4;
    localparam [3:0] S_ADDRESS   = 4'd5;
    localparam [3:0] S_DUMMY     = 4'd6;
    localparam [3:0] S_READ      = 4'd7;
    localparam [3:0] S_WRITE     = 4'd8;
    localparam [3:0] S_PAGE_GAP  = 4'd9;
    localparam [3:0] S_END_GAP   = 4'd10;

    reg [3:0] state;
    reg       init_after_f5;
    reg [1:0] gap_count;
    reg       nibble_phase;
    reg [2:0] init_count;
    reg [2:0] address_count;
    reg [2:0] dummy_count;
    reg [17:0] current_addr;
    reg       operation_we;
    reg [3:0] data_nibble;
    reg       data_oe;

    wire [17:0] next_addr = current_addr + 18'd1;

    assign busy = (state != S_IDLE);

    always @* begin
        data_nibble = 4'd0;
        sd_out = 4'd0;
        sd_oe = 4'd0;
        data_oe = 1'b0;

        case (state)
            S_INIT_F5: begin
                data_nibble = (init_count == 3'd0) ? 4'hf : 4'h5;
                data_oe = 1'b1;
            end
            S_INIT_35: begin
                case (init_count)
                    3'd0: sd_out[0] = 1'b0;
                    3'd1: sd_out[0] = 1'b0;
                    3'd2: sd_out[0] = 1'b1;
                    3'd3: sd_out[0] = 1'b1;
                    3'd4: sd_out[0] = 1'b0;
                    3'd5: sd_out[0] = 1'b1;
                    3'd6: sd_out[0] = 1'b0;
                    default: sd_out[0] = 1'b1;
                endcase
                sd_oe[0] = 1'b1;
            end
            S_COMMAND: begin
                data_nibble = (init_count == 3'd0) ? 4'd0 :
                              (operation_we ? 4'h2 : 4'hb);
                data_oe = 1'b1;
            end
            S_ADDRESS: begin
                case (address_count)
                    3'd0: data_nibble = 4'd0;
                    3'd1: data_nibble = {2'b00, current_addr[17:16]};
                    3'd2: data_nibble = current_addr[15:12];
                    3'd3: data_nibble = current_addr[11:8];
                    3'd4: data_nibble = current_addr[7:4];
                    default: data_nibble = current_addr[3:0];
                endcase
                data_oe = 1'b1;
            end
            S_WRITE: begin
                data_nibble = nibble_phase ? wdata[3:0] : wdata[7:4];
                data_oe = 1'b1;
            end
            default: begin
                data_nibble = 4'd0;
            end
        endcase

        if (data_oe) begin
            sd_out = data_nibble;
            sd_oe = 4'b1111;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_INIT_F5;
            psram_cs_n <= 1'b1;
            psram_sck <= 1'b0;
            init_after_f5 <= 1'b0;
            gap_count <= 2'd0;
            nibble_phase <= 1'b0;
            init_count <= 3'd0;
            address_count <= 3'd0;
            dummy_count <= 3'd0;
            current_addr <= 18'd0;
            operation_we <= 1'b0;
            ready <= 1'b0;
            bdone <= 1'b0;
            rdata <= 8'd0;
        end else begin
            bdone <= 1'b0;

            case (state)
                S_INIT_F5: begin
                    if (psram_cs_n) begin
                        psram_cs_n <= 1'b0;
                        psram_sck <= 1'b0;
                        init_count <= 3'd0;
                    end else if (!psram_sck) begin
                        psram_sck <= 1'b1;
                    end else begin
                        psram_sck <= 1'b0;
                        if (init_count == 3'd1) begin
                            psram_cs_n <= 1'b1;
                            state <= S_INIT_GAP;
                            init_after_f5 <= 1'b1;
                            gap_count <= 2'd0;
                        end else begin
                            init_count <= 3'd1;
                        end
                    end
                end

                S_INIT_GAP: begin
                    if (gap_count < 2'd2) begin
                        gap_count <= gap_count + 2'd1;
                    end else if (init_after_f5) begin
                        state <= S_INIT_35;
                        psram_cs_n <= 1'b0;
                        psram_sck <= 1'b0;
                        init_count <= 3'd0;
                        init_after_f5 <= 1'b0;
                    end else begin
                        state <= S_IDLE;
                        ready <= 1'b1;
                    end
                end

                S_INIT_35: begin
                    if (!psram_sck) begin
                        psram_sck <= 1'b1;
                    end else begin
                        psram_sck <= 1'b0;
                        if (init_count == 3'd7) begin
                            psram_cs_n <= 1'b1;
                            state <= S_INIT_GAP;
                            gap_count <= 2'd0;
                        end else begin
                            init_count <= init_count + 3'd1;
                        end
                    end
                end

                S_IDLE: begin
                    if (start && ready) begin
                        state <= S_COMMAND;
                        psram_cs_n <= 1'b0;
                        psram_sck <= 1'b0;
                        current_addr <= addr;
                        operation_we <= we;
                        nibble_phase <= 1'b0;
                        init_count <= 3'd0;
                        address_count <= 3'd0;
                    end
                end

                S_COMMAND: begin
                    if (!psram_sck) begin
                        psram_sck <= 1'b1;
                    end else begin
                        psram_sck <= 1'b0;
                        if (init_count == 3'd0) begin
                            init_count <= 3'd1;
                        end else begin
                            state <= S_ADDRESS;
                            address_count <= 3'd0;
                        end
                    end
                end

                S_ADDRESS: begin
                    if (!psram_sck) begin
                        psram_sck <= 1'b1;
                    end else begin
                        psram_sck <= 1'b0;
                        if (address_count == 3'd5) begin
                            if (operation_we) begin
                                state <= S_WRITE;
                                nibble_phase <= 1'b0;
                            end else begin
                                state <= S_DUMMY;
                                dummy_count <= 3'd0;
                            end
                        end else begin
                            address_count <= address_count + 3'd1;
                        end
                    end
                end

                S_DUMMY: begin
                    if (!psram_sck) begin
                        psram_sck <= 1'b1;
                    end else begin
                        psram_sck <= 1'b0;
                        if (dummy_count == 3'd3) begin
                            state <= S_READ;
                            nibble_phase <= 1'b0;
                        end else begin
                            dummy_count <= dummy_count + 3'd1;
                        end
                    end
                end

                S_READ: begin
                    if (!psram_sck) begin
                        psram_sck <= 1'b1;
                        if (!rd_late) begin
                            if (!nibble_phase)
                                rdata[7:4] <= sd_in;
                            else begin
                                rdata[3:0] <= sd_in;
                                bdone <= 1'b1;
                            end
                        end
                    end else begin
                        psram_sck <= 1'b0;
                        if (rd_late) begin
                            if (!nibble_phase)
                                rdata[7:4] <= sd_in;
                            else begin
                                rdata[3:0] <= sd_in;
                                bdone <= 1'b1;
                            end
                        end

                        if (!nibble_phase) begin
                            nibble_phase <= 1'b1;
                        end else if (more) begin
                            if (current_addr[9:0] == 10'h3ff) begin
                                current_addr <= next_addr;
                                psram_cs_n <= 1'b1;
                                state <= S_PAGE_GAP;
                                gap_count <= 2'd0;
                            end else begin
                                current_addr <= next_addr;
                                nibble_phase <= 1'b0;
                            end
                        end else begin
                            psram_cs_n <= 1'b1;
                            state <= S_END_GAP;
                            gap_count <= 2'd0;
                        end
                    end
                end

                S_WRITE: begin
                    if (!psram_sck) begin
                        psram_sck <= 1'b1;
                        if (nibble_phase)
                            bdone <= 1'b1;
                    end else begin
                        psram_sck <= 1'b0;
                        if (!nibble_phase) begin
                            nibble_phase <= 1'b1;
                        end else if (more) begin
                            if (current_addr[9:0] == 10'h3ff) begin
                                current_addr <= next_addr;
                                psram_cs_n <= 1'b1;
                                state <= S_PAGE_GAP;
                                gap_count <= 2'd0;
                            end else begin
                                current_addr <= next_addr;
                                nibble_phase <= 1'b0;
                            end
                        end else begin
                            psram_cs_n <= 1'b1;
                            state <= S_END_GAP;
                            gap_count <= 2'd0;
                        end
                    end
                end

                S_PAGE_GAP: begin
                    if (gap_count < 2'd2) begin
                        gap_count <= gap_count + 2'd1;
                    end else begin
                        state <= S_COMMAND;
                        psram_cs_n <= 1'b0;
                        psram_sck <= 1'b0;
                        init_count <= 3'd0;
                        address_count <= 3'd0;
                    end
                end

                S_END_GAP: begin
                    if (gap_count < 2'd2)
                        gap_count <= gap_count + 2'd1;
                    else
                        state <= S_IDLE;
                end

                default: begin
                    state <= S_INIT_F5;
                    psram_cs_n <= 1'b1;
                    psram_sck <= 1'b0;
                    ready <= 1'b0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
