`default_nettype none

module pca9635_master #(
    parameter LED_CFG  = "",
    parameter LED_CFG_BYTES = 16'd26
)(
    input  clk,
    input  rst,

    // I2C signals to be routed to IO.
	output scl_oe,
	input  scl_i,
	output sda_oe,
	input  sda_i,

    input [7:0] led0,
    input [7:0] led1,
    input [7:0] led2,
    input [7:0] led3,
    input [7:0] led4,
    input [7:0] led5,
    input [7:0] led6,
    input [7:0] led7,
    input [7:0] led8,
    input [7:0] led9,
    input [7:0] led10,
    input [7:0] led11,
    input [7:0] led12,
    input [7:0] led13,
    input [7:0] led14,
    input [7:0] led15,
);

// Overall state machine of this core.
localparam I2C_LED1         = 0,
           I2C_LED2         = 1;


logic [3:0] i2c_state = I2C_LED1;

// Index into i2c config memories
logic [15:0] i2c_config_pos = 0;

// Logic for startup configuration of LEDs over I2C.
logic [7:0] led_config [0:LED_CFG_BYTES-1];
initial $readmemh(LED_CFG, led_config);

// Index at which PWM values start in the led config.
localparam PCA9635_PWM0 = 4;

// Valid commands for `i2c_master` core.
localparam [1:0] I2CMASTER_START = 2'b00,
                 I2CMASTER_STOP  = 2'b01,
                 I2CMASTER_WRITE = 2'b10,
                 I2CMASTER_READ  = 2'b11;

// Outbound signals to `i2c_master` core.
logic [7:0] data_in;
logic       ack_in;
logic [1:0] cmd;
logic       stb = 1'b0;

// Inbound signals from `i2c_master` core.
logic [7:0] data_out;
logic       ack_out;
logic       err_out;
logic       ready;

always_ff @(posedge clk) begin
    if (rst) begin
        i2c_state <= I2C_LED1;
    end else begin
        if (ready && ~stb) begin
            case (i2c_state)
                I2C_LED1: begin
                    cmd <= I2CMASTER_START;
                    stb <= 1'b1;
                    i2c_state <= I2C_LED2;
                    i2c_config_pos <= 0;
                end
                I2C_LED2: begin
                    case (i2c_config_pos)
                        LED_CFG_BYTES: begin
                            cmd <= I2CMASTER_STOP;
                            i2c_state <= I2C_LED1;
                        end
                        default: begin
                            data_in <= led_config[5'(i2c_config_pos)];
                            cmd <= I2CMASTER_WRITE;
                        end
                        // Override PWM values from led configuration.
                        PCA9635_PWM0 +  0: data_in <= led0;
                        PCA9635_PWM0 +  1: data_in <= led1;
                        PCA9635_PWM0 +  2: data_in <= led2;
                        PCA9635_PWM0 +  3: data_in <= led3;
                        PCA9635_PWM0 +  4: data_in <= led4;
                        PCA9635_PWM0 +  5: data_in <= led5;
                        PCA9635_PWM0 +  6: data_in <= led6;
                        PCA9635_PWM0 +  7: data_in <= led7;
                        PCA9635_PWM0 +  8: data_in <= led8;
                        PCA9635_PWM0 +  9: data_in <= led9;
                        PCA9635_PWM0 + 10: data_in <= led10;
                        PCA9635_PWM0 + 11: data_in <= led11;
                        PCA9635_PWM0 + 12: data_in <= led12;
                        PCA9635_PWM0 + 13: data_in <= led13;
                        PCA9635_PWM0 + 14: data_in <= led14;
                        PCA9635_PWM0 + 15: data_in <= led15;
                    endcase
                    i2c_config_pos <= i2c_config_pos + 1;
                    ack_in <= 1'b1;
                    stb <= 1'b1;
                end
            endcase
        end else begin
            stb <= 1'b0;
        end
    end
end

i2c_master #(.DW(4)) i2c_master_inst(
    .scl_oe(scl_oe),
    .scl_i(scl_i),
    .sda_oe(sda_oe),
    .sda_i(sda_i),

    .data_in(data_in),
    .ack_in(ack_in),
    .cmd(cmd),
    .stb(stb),

    .data_out(data_out),
    .ack_out(ack_out),
    .err_out(err_out),

    .ready(ready),

    .clk(clk),
    .rst(rst)
);

endmodule
