module alu #(
    parameter INST_W = 4,
    parameter INT_W  = 6,
    parameter FRAC_W = 10,
    parameter DATA_W = INT_W + FRAC_W
)(
    input                      i_clk,
    input                      i_rst_n,

    input                      i_in_valid,
    output reg                 o_busy,
    input         [INST_W-1:0] i_inst,
    input  signed [DATA_W-1:0] i_data_a, // Q6.10
    input  signed [DATA_W-1:0] i_data_b, // Q6.10

    output reg                 o_out_valid,
    output reg    [DATA_W-1:0] o_data
);


localparam LOAD = 1'd0, OUTPUT = 1'd1;

parameter [15:0] MAX_VAL = 16'h7FFF;
parameter [15:0] MIN_VAL = 16'h8000;
parameter signed [15:0] TAYLOR_COEF_1 = 16'b0000_0000_1010_1011;   // 1/6 in Q6.10
parameter signed [15:0] TAYLOR_COEF_2 = 16'b1111_1111_1111_0111;   // -1/120 in Q6.10
parameter signed [15:0] ONE = 16'sh0400;                           // 1.0 in Q6.10

reg curr_state, next_state;
reg [2:0] cnt;

reg o_busy_w;
reg o_out_valid_w;
reg [15:0] o_data_w;

wire signed [16:0] add_result;
wire signed [16:0] sub_result;
wire signed [31:0] mult_result;
reg signed [35:0] data_acc;    // Q16.20
wire signed [36:0] acc_sum;
reg signed [35:0] acc_sat;
reg signed [16:0] acc_rounded;

reg signed [31:0] x2_temp;    // Q12.20
reg signed [15:0] x2;         // Q6.10
reg signed [31:0] term1_temp; // Q12.20
reg signed [15:0] term1;      // Q6.10
reg signed [15:0] term2;      // Q6.10
reg signed [31:0] term3_temp; // Q12.20
reg signed [15:0] term3;      // Q6.10
reg signed [15:0] term4;      // Q6.10
reg signed [31:0] final_result;


wire [3:0] CPOP;
reg [DATA_W-1:0] LRCW_w;
wire [2*DATA_W-1:0] r_rot_w;
reg stop; // for count leading 0s
reg [15:0] matrix_transposed [0:7];

integer i;


// Adder and Subtractor 
assign add_result = i_data_a + i_data_b; // Q7.10
assign sub_result = i_data_a - i_data_b; // Q7.10


// Multiplication & Accumulation (with Saturation)
assign mult_result = i_data_a * i_data_b; // Q12.20
assign acc_sum = mult_result + data_acc;  // Q17.20


// CPOP encoder
assign CPOP = i_data_a[0] + i_data_a[1] + i_data_a[2] + i_data_a[3] + 
                i_data_a[4] + i_data_a[5] + i_data_a[6] + i_data_a[7] + 
                i_data_a[8] + i_data_a[9] + i_data_a[10] + i_data_a[11] +
                i_data_a[12] + i_data_a[13] + i_data_a[14] + i_data_a[15];


// For Right Rotation
assign r_rot_w = {i_data_a, i_data_a};


// Accumulator (36bits)
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        data_acc <= 36'd0;
    end else if (i_inst == 4'b0010)begin
        data_acc <= acc_sat;
    end
end


// Matrix
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        for (i = 0; i < 8; i = i + 1)
            matrix_transposed[i] <= 16'd0;
    end else if (curr_state == LOAD) begin
        for (i = 0; i < 8; i = i + 1) begin
            matrix_transposed[i][15 - 2*cnt -: 2] <= i_data_a[15 - 2*i -: 2];
        end
    end
end


// Counter
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        cnt <= 3'd0;
    end else if (i_inst == 4'b1001 && curr_state == LOAD) begin
        if (i_in_valid)
            cnt <= (cnt == 3'd7) ? 3'd0 : cnt + 1;
    end else if (curr_state == OUTPUT) begin
        cnt <= (cnt == 3'd7) ? 3'd0 : cnt + 1;
    end
end


// FSM (for matrix transpose)
// CS
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) 
        curr_state <= LOAD;
    else 
        curr_state <= next_state;
end

// NS
always @(*) begin
    case (curr_state)
        LOAD: next_state = (cnt == 3'd7 && i_in_valid) ? OUTPUT : LOAD;
        OUTPUT: next_state = (cnt == 3'd7) ? LOAD : OUTPUT;
    endcase
end


// OL
always @(*) begin
    o_data_w = 16'd0;
    o_busy_w = 1'b0;
    o_out_valid_w = 1'b0;
    acc_sat = 36'sd0;
    acc_rounded = 17'sd0;
    LRCW_w = 32'd0;
    stop = 1'b0;

    x2_temp = 32'd0;
    x2 = 16'd0;
    term1_temp = 32'd0;
    term1 = 16'd0;
    term2 = 16'd0;
    term3_temp = 32'd0;
    term3 = 16'd0;
    term4 = 16'd0;
    final_result = 32'd0;

    
    case (i_inst)
        // Signed Addition with Saturation
        4'b0000: begin 
            if (i_in_valid) begin
                if ((i_data_a[15] == i_data_b[15]) && (add_result[15] != i_data_a[15]))
                    o_data_w = i_data_a[15] ? MIN_VAL : MAX_VAL;
                else
                    o_data_w = add_result[DATA_W-1:0];
                            
                o_out_valid_w = 1'b1;
                o_busy_w = 0;
            end
        end

        // Signed Subtraction with Saturation
        4'b0001: begin 
            if (i_in_valid) begin
                if ((i_data_a[15] != i_data_b[15]) && (sub_result[15] != i_data_a[15]))
                    o_data_w = i_data_a[15] ? MIN_VAL : MAX_VAL;
                else
                    o_data_w = sub_result[DATA_W-1:0];    

                o_out_valid_w = 1'b1;
                o_busy_w = 0;
            end
        end

        // Signed Multiplication and Accumulation
        4'b0010: begin 
            if (i_in_valid) begin

                acc_sat = (acc_sum[36] && !mult_result[31] && !data_acc[35]) ? 36'sh7_FFFF_FFFF : //兩個正加起來為負的
                          (!acc_sum[36] && mult_result[31] && data_acc[35]) ? 36'sh8_0000_0000 :  //兩個負加起來為正的
                           acc_sum[35:0];

                o_data_w = acc_sat[25:10] + acc_sat[9]; // Rounding
                        
                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end

        // Taylor Expansion of Sin Function
        // By Horner's Rule: x + 1/6 * x^3 + 1/120 * x^5 = x * (1 - x^2 * (1/6 - 1/120 * x^2)) )
        4'b0011: begin 
            if (i_in_valid) begin
                // x^2
                x2_temp = i_data_a * i_data_a;   // Q12.20
                x2 = x2_temp[25:10]+ x2_temp[9]; // Q6.10 (with rounding)
                // -1/120 * x^2 
                term1_temp = x2 * TAYLOR_COEF_2;           // Q12.20
                term1 = term1_temp[25:10] + term1_temp[9]; // Q6.10 (with rounding)
                // 1/6? - 1/120 * x^2
                term2 = TAYLOR_COEF_1 + term1;   // Q6.10
                // x^2 * (1/6? - 1/120 * x^2 ?)
                term3_temp = x2 * term2;                   // Q12.20
                term3 = term3_temp[25:10] + term3_temp[9]; // Q6.10 (with rounding)
                // 1 ? x^2 * (1/6? - 1/120 * x^2 ?)
                term4 = ONE - term3;
                // Final Result: x * (1 ? x^2 * (1/6? - 1/120 * x^2 ?))
                final_result = i_data_a * term4;
                o_data_w = final_result[25:10] + final_result[9];

                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end

        // Binary to Gray Code
        4'b0100: begin
            if (i_in_valid) begin
                for (i = DATA_W-1; i >= 0; i = i-1) begin
                    if (i == DATA_W - 1)
                        o_data_w[i] = i_data_a[i];
                    else
                        o_data_w[i] = i_data_a[i+1] ^ i_data_a[i];
                end

                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end

        // LRCW
        4'b0101: begin
            if (i_in_valid) begin
                {o_data_w, LRCW_w} = {i_data_b,~i_data_b} << CPOP;

                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end

        // Right Rotation
        4'b0110: begin
            if (i_in_valid) begin
                o_data_w = r_rot_w[i_data_b[3:0] +: 16]; // i_data_b is guaranteed to be from 0 to 16 

                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end

        // Count Leading Zeros 
        4'b0111: begin
            if (i_in_valid) begin
                o_data_w = 16'd16;
                for (i = 15; i >= 0; i = i-1) begin
                    if (i_data_a[i] && !stop) begin
                        o_data_w = 16'd15 - i;
                        stop = 1'b1;
                    end
                end

                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end

        // Reverse Match4
        4'b1000: begin
            if (i_in_valid) begin
                for (i = 0; i <= 12; i = i+1) begin
                    o_data_w[i] = (i_data_a[i +: 4] == i_data_b[15-i -: 4]);
                end
                o_data_w[15:13] = 3'd0;

                o_out_valid_w = 1'b1;
                o_busy_w = 1'b0;
            end
        end
        
        // Matrix Transpose
        4'b1001: begin
            if (curr_state == LOAD) begin
                o_out_valid_w = 1'b0;
                o_busy_w = (cnt == 7) ? 1'b1 : 1'b0;
            end
        end

        default: begin end
    endcase
    
end



// Output registers
always @(posedge i_clk or negedge i_rst_n) begin
    if (!i_rst_n) begin
        o_data      <= 1'b0;
        o_busy      <= 1'b0;
        o_out_valid <= 1'b0;
    end else if (curr_state == OUTPUT) begin // output matrix transpose result
        o_data      <= matrix_transposed[cnt];
        o_busy      <= 1'b1;
        o_out_valid <= 1'b1;
    end else begin
        o_data      <= o_data_w;
        o_busy      <= o_busy_w;
        o_out_valid <= o_out_valid_w;
    end 
end


endmodule
