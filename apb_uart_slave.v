// =============================================================
// APB UART - decoupled TX/RX design
//   Reg 0x0 : DATA   (W: TX data, load & start transmit)
//                     (R: last received byte, clears rx_valid)
//   Reg 0x4 : STATUS (R: bit0 = tx_busy, bit1 = rx_valid)
// =============================================================
module apb_uart #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115_200
) (
    input  wire         pclk,
    input  wire         presetn,   // active-low APB reset (single reset used)

    // UART pins
    input  wire         rx,
    output reg           tx,

    // APB slave interface
    input  wire         psel,
    input  wire         penable,
    input  wire         pwrite,
    input  wire [31:0]  paddr,
    input  wire [7:0]   pwdata,
    output reg  [7:0]   prdata,
    output reg          pready
);

    localparam integer BAUD_DIV = CLK_FREQ / BAUD_RATE;

    // -----------------------------------------------------------
    // Baud tick generator (free-running, 1 tick per bit period)
    // -----------------------------------------------------------
    reg [$clog2(BAUD_DIV):0] baud_cnt;
    reg                      baud_tick;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            baud_cnt  <= 0;
            baud_tick <= 1'b0;
        end else if (baud_cnt == BAUD_DIV - 1) begin
            baud_cnt  <= 0;
            baud_tick <= 1'b1;
        end else begin
            baud_cnt  <= baud_cnt + 1'b1;
            baud_tick <= 1'b0;
        end
    end

    // -----------------------------------------------------------
    // RX synchronizer (avoid metastability on async rx pin)
    // -----------------------------------------------------------
    reg rx_sync0, rx_sync1;
    always @(posedge pclk or negedge presetn) begin
        if (!presetn) {rx_sync1, rx_sync0} <= 2'b11;
        else          {rx_sync1, rx_sync0} <= {rx_sync0, rx};
    end
    wire rx_s = rx_sync1;

    // -----------------------------------------------------------
    // TX FSM
    // -----------------------------------------------------------
    localparam TX_IDLE = 2'd0, TX_START = 2'd1, TX_DATA = 2'd2, TX_STOP = 2'd3;
    reg [1:0] tx_state;
    reg [7:0] tx_shift;
    reg [2:0] tx_bitcnt;
    reg       tx_busy;
    reg       tx_start_req;   // pulsed by APB write
    reg [7:0] tx_wdata;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            tx        <= 1'b1;     // idle line = high
            tx_state  <= TX_IDLE;
            tx_busy   <= 1'b0;
            tx_bitcnt <= 0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    tx <= 1'b1;
                    if (tx_start_req) begin
                        tx_shift <= tx_wdata;
                        tx_busy  <= 1'b1;
                        tx_state <= TX_START;
                    end
                end

                TX_START: if (baud_tick) begin
                    tx        <= 1'b0;   // start bit
                    tx_bitcnt <= 0;
                    tx_state  <= TX_DATA;
                end

                TX_DATA: if (baud_tick) begin
                    tx <= tx_shift[tx_bitcnt];
                    if (tx_bitcnt == 3'd7)
                        tx_state <= TX_STOP;
                    else
                        tx_bitcnt <= tx_bitcnt + 1'b1;
                end

                TX_STOP: if (baud_tick) begin
                    tx       <= 1'b1;   // stop bit
                    tx_busy  <= 1'b0;
                    tx_state <= TX_IDLE;
                end

                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // -----------------------------------------------------------
    // RX FSM (samples at mid-bit using a half-period offset)
    // -----------------------------------------------------------
    localparam RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;
    reg [1:0]                      rx_state;
    reg [7:0]                      rx_shift;
    reg [2:0]                      rx_bitcnt;
    reg                            rx_valid;
    reg [7:0]                      rxdata_reg;
    reg [$clog2(BAUD_DIV):0]       rx_cnt;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            rx_state  <= RX_IDLE;
            rx_valid  <= 1'b0;
            rx_bitcnt <= 0;
            rx_cnt    <= 0;
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    rx_cnt <= 0;
                    if (rx_s == 1'b0)          // possible start bit
                        rx_state <= RX_START;
                end

                RX_START: begin
                    // wait half a bit period, then confirm still low
                    if (rx_cnt == (BAUD_DIV/2)) begin
                        rx_cnt <= 0;
                        if (rx_s == 1'b0) begin
                            rx_state  <= RX_DATA;
                            rx_bitcnt <= 0;
                        end else begin
                            rx_state  <= RX_IDLE;   // glitch, not a real start
                        end
                    end else begin
                        rx_cnt <= rx_cnt + 1'b1;
                    end
                end

                RX_DATA: begin
                    if (rx_cnt == BAUD_DIV - 1) begin
                        rx_cnt          <= 0;
                        rx_shift[rx_bitcnt] <= rx_s;
                        if (rx_bitcnt == 3'd7)
                            rx_state <= RX_STOP;
                        else
                            rx_bitcnt <= rx_bitcnt + 1'b1;
                    end else begin
                        rx_cnt <= rx_cnt + 1'b1;
                    end
                end

                RX_STOP: begin
                    if (rx_cnt == BAUD_DIV - 1) begin
                        rx_cnt <= 0;
                        if (rx_s == 1'b1) begin   // valid stop bit
                            rxdata_reg <= rx_shift;
                            rx_valid   <= 1'b1;
                        end
                        // else: framing error - byte dropped
                        rx_state <= RX_IDLE;
                    end else begin
                        rx_cnt <= rx_cnt + 1'b1;
                    end
                end

                default: rx_state <= RX_IDLE;
            endcase

            // clear rx_valid when CPU reads DATA reg (handled in APB block below)
            if (apb_rx_read_pulse)
                rx_valid <= 1'b0;
        end
    end

    // -----------------------------------------------------------
    // APB slave interface (single wait-state, address-decoded)
    // -----------------------------------------------------------
    localparam ADDR_DATA   = 4'h0;
    localparam ADDR_STATUS = 4'h4;

    wire        apb_access = psel && penable;
    reg         apb_rx_read_pulse;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            pready       <= 1'b0;
            prdata       <= 8'h00;
            tx_start_req <= 1'b0;
            tx_wdata     <= 8'h00;
            apb_rx_read_pulse <= 1'b0;
        end else begin
            tx_start_req      <= 1'b0;   // default: single-cycle pulse
            apb_rx_read_pulse <= 1'b0;

            if (apb_access && !pready) begin
                pready <= 1'b1;

                if (pwrite) begin
                    case (paddr[3:0])
                        ADDR_DATA: if (!tx_busy) begin
                            tx_wdata     <= pwdata;
                            tx_start_req <= 1'b1;
                        end
                        default: ; // ignore writes to STATUS / unmapped
                    endcase
                end else begin
                    case (paddr[3:0])
                        ADDR_DATA: begin
                            prdata            <= rxdata_reg;
                            apb_rx_read_pulse <= 1'b1;
                        end
                        ADDR_STATUS: prdata <= {6'b0, rx_valid, tx_busy};
                        default:     prdata <= 8'h00;
                    endcase
                end
            end else begin
                pready <= 1'b0;
            end
        end
    end

endmodule
