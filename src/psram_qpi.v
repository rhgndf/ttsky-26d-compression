`default_nettype none

module psram_qpi (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rd_late,
    input  wire       start,
    input  wire       we,
    input  wire [17:0] addr,
    input  wire       more,
    input  wire [3:0]  wnib,
    output reg        psram_sck,
    output reg        psram_cs_n,
    output reg  [3:0] sd_out,
    output reg  [3:0] sd_oe,
    output reg        ready,
    output wire       busy,
    output wire       nib_strobe,
    output wire       byte_done
);
    localparam [3:0] S_INIT=0, S_INIT_F5=1, S_INIT_35_START=2;
    localparam [3:0] S_INIT_35=3, S_IDLE=4, S_CMD=5, S_ADDR=6;
    localparam [3:0] S_DUMMY=7, S_DATA=8, S_GAP=9;
    localparam [3:0] S_DATA_FALL=10, S_DATA_DECIDE=11;
    reg [3:0] state, phase;
    reg nib;
    reg [1:0] gap;
    reg [1:0] init_stage;
    reg operation_we;
    reg more_q;
    reg page_q;
    reg restart_q;
    assign busy = (state != S_IDLE);
    assign nib_strobe = (state == S_DATA) &&
                        (operation_we ? psram_sck :
                         (rd_late ? psram_sck : !psram_sck));
    assign byte_done = nib_strobe && nib;

    always @* begin
        sd_out = 0;
        sd_oe = 0;
        if (state == S_INIT_F5) begin
            sd_out = (phase == 0) ? 4'hf : 4'h5;
            sd_oe = 4'hf;
        end else if (state == S_INIT_35) begin
            sd_out[0] = ((8'h35 & (8'h80 >> phase)) != 0);
            sd_oe = 4'b0001;
        end else if (state == S_CMD) begin
            sd_out = (phase == 0) ? (operation_we ? 0 : 4'he) :
                                     (operation_we ? 4'h2 : 4'hb);
            sd_oe = 4'hf;
        end else if (state == S_ADDR) begin
            case (phase)
                0: sd_out = 0;
                1: sd_out = {2'b00, addr[17:16]};
                2: sd_out = addr[15:12];
                3: sd_out = addr[11:8];
                4: sd_out = addr[7:4];
                default: sd_out = addr[3:0];
            endcase
            sd_oe = 4'hf;
        end else if (state == S_DATA && operation_we) begin
            sd_out = wnib;
            sd_oe = 4'hf;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_INIT;
            phase <= 0;
            nib <= 0;
            gap <= 0;
            init_stage <= 0;
            operation_we <= 0;
            more_q <= 0;
            page_q <= 0;
            restart_q <= 0;
            psram_sck <= 0;
            psram_cs_n <= 1;
            ready <= 0;
        end else begin
            case (state)
                S_INIT: begin
                    psram_cs_n <= 1;
                    psram_sck <= 0;
                    phase <= 0;
                    psram_cs_n <= 0;
                    state <= S_INIT_F5;
                end
                S_INIT_F5: begin
                    if (!psram_sck)
                        psram_sck <= 1;
                    else begin
                        psram_sck <= 0;
                        if (phase == 1) begin
                            phase <= 0;
                            psram_cs_n <= 1;
                            gap <= 0;
                            init_stage <= 1;
                            state <= S_GAP;
                        end else phase <= phase + 1'b1;
                    end
                end
                S_INIT_35_START: begin
                    psram_cs_n <= 0;
                    psram_sck <= 0;
                    phase <= 0;
                    state <= S_INIT_35;
                end
                S_INIT_35: begin
                    if (!psram_sck)
                        psram_sck <= 1;
                    else begin
                        psram_sck <= 0;
                        if (phase == 7) begin
                            phase <= 0;
                            psram_cs_n <= 1;
                            gap <= 0;
                            state <= S_GAP;
                        end else phase <= phase + 1'b1;
                    end
                end
                S_IDLE: begin
                    psram_cs_n <= 1;
                    psram_sck <= 0;
                    if (start) begin
                        operation_we <= we;
                        phase <= 0;
                        nib <= 0;
                        restart_q <= 0;
                        psram_cs_n <= 0;
                        state <= S_CMD;
                    end
                end
                S_CMD: begin
                    if (!psram_sck)
                        psram_sck <= 1;
                    else begin
                        psram_sck <= 0;
                        if (phase == 1) begin phase <= 0; state <= S_ADDR; end
                        else phase <= 1;
                    end
                end
                S_ADDR: begin
                    if (!psram_sck)
                        psram_sck <= 1;
                    else begin
                        psram_sck <= 0;
                        if (phase == 5) begin
                            phase <= 0;
                            if (operation_we) state <= S_DATA;
                            else state <= S_DUMMY;
                        end else phase <= phase + 1'b1;
                    end
                end
                S_DUMMY: begin
                    if (!psram_sck)
                        psram_sck <= 1;
                    else begin
                        psram_sck <= 0;
                        if (phase == 5) begin phase <= 0; nib <= 0; state <= S_DATA; end
                        else phase <= phase + 1'b1;
                    end
                end
                S_DATA: begin
                    if (byte_done) begin
                        more_q <= more;
                        page_q <= more && (addr[9:0] == 10'h3ff);
                        if (psram_sck) begin
                            psram_sck <= 0;
                            state <= S_DATA_DECIDE;
                        end else begin
                            psram_sck <= 1;
                            state <= S_DATA_FALL;
                        end
                    end else if (!psram_sck) begin
                        psram_sck <= 1;
                    end else begin
                        psram_sck <= 0;
                        if (!nib)
                            nib <= 1;
                    end
                end
                S_DATA_FALL: begin
                    psram_sck <= 0;
                    state <= S_DATA_DECIDE;
                end
                S_DATA_DECIDE: begin
                    if (more_q) begin
                        if (page_q) begin
                            psram_cs_n <= 1;
                            gap <= 0;
                            restart_q <= 1;
                            state <= S_GAP;
                        end else begin
                            nib <= 0;
                            state <= S_DATA;
                        end
                    end else begin
                        psram_cs_n <= 1;
                        gap <= 0;
                        restart_q <= 0;
                        state <= S_GAP;
                    end
                end
                S_GAP: begin
                    psram_cs_n <= 1;
                    psram_sck <= 1;
                    if (gap == 2) begin
                        if (init_stage == 1) begin
                            init_stage <= 2;
                            state <= S_INIT_35_START;
                            psram_sck <= 0;
                        end else if (init_stage == 2) begin
                            init_stage <= 0;
                            ready <= 1;
                            state <= S_IDLE;
                            psram_sck <= 0;
                        end else if (restart_q) begin
                            phase <= 0;
                            nib <= 0;
                            psram_cs_n <= 0;
                            psram_sck <= 0;
                            state <= S_CMD;
                            restart_q <= 0;
                        end else begin
                            ready <= 1;
                            state <= S_IDLE;
                            psram_sck <= 0;
                        end
                    end else gap <= gap + 1'b1;
                end
                default: state <= S_INIT;
            endcase
        end
    end
endmodule

`default_nettype wire
