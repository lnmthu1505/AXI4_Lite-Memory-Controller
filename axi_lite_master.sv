`timescale 1ns/1ps

module axi_lite_master #
   (
    parameter C_M_AXI_ADDR_WIDTH    = 32,
    parameter C_M_AXI_DATA_WIDTH    = 32,
    parameter C_M_AXI_BIT_WIDTH     = 3,
    parameter C_NUM_COMMANDS        = 3,
    parameter BASE_ADDR             = 32'h10000080
    )
   (
    // System Signals
    input wire M_AXI_ACLK,
    input wire M_AXI_ARESETN,
    
    //input gpio 
    input i_gpio,

    // Master Interface Write Address
    output  [C_M_AXI_ADDR_WIDTH-1:0]        M_AXI_AWADDR,
    output  [C_M_AXI_BIT_WIDTH-1:0]         M_AXI_AWPROT,
    output  M_AXI_AWVALID,
    input   M_AXI_AWREADY,

    // Master Interface Write Data
    output  [C_M_AXI_DATA_WIDTH-1:0]        M_AXI_WDATA,
    output  [C_M_AXI_BIT_WIDTH/8-1:0]       M_AXI_WSTRB,
    output  M_AXI_WVALID,
    input   M_AXI_WREADY,

    // Master Interface Write Response
    input   [C_M_AXI_BIT_WIDTH/2:0]         M_AXI_BRESP,
    input   M_AXI_BVALID,
    output  M_AXI_BREADY,

    // Master Interface Read Address
    output  [C_M_AXI_ADDR_WIDTH-1:0]        M_AXI_ARADDR,
    output  [C_M_AXI_BIT_WIDTH-1:0]         M_AXI_ARPROT,
    output  M_AXI_ARVALID,
    input   M_AXI_ARREADY,

    // Master Interface Read Data 
    input   [C_M_AXI_DATA_WIDTH-1:0]        M_AXI_RDATA,
    input   [C_M_AXI_BIT_WIDTH/2:0]         M_AXI_RRESP,
    input   M_AXI_RVALID,
    output  M_AXI_RREADY
    );
    
    //AXI4 signals
    logic 		  awvalid, arvalid, wvalid;
    logic 		  wpush, rpop;
    logic         wr_done, rd_done;
    logic         rready, bready;
    logic [C_M_AXI_ADDR_WIDTH-1:0] 	awaddr, araddr;
    
    logic 	      wrsp_error;
    logic         rrsp_error;
    logic         rd_mismatched;
    logic         error_tick;
    logic         suc_fin;
    logic [C_M_AXI_DATA_WIDTH-1:0]	rdata, wdata;
    
    //specific register
    logic [C_M_AXI_DATA_WIDTH-1:0]  wr_id, rd_id;
    
    /////////////////////
    //Write adress (AW)//
    /////////////////////
    assign M_AXI_AWADDR = awaddr;
    assign M_AXI_AWVALID = awvalid;
    assign M_AXI_AWPROT = 3'b0;
    /////////////////////
    //Write data (W)////
    ////////////////////
    assign M_AXI_WDATA = wdata;
    assign M_AXI_WVALID = wvalid;
    assign M_AXI_WSTRB = -1;
    //////////////////////
    //Read address (AR)//
    /////////////////////
    assign M_AXI_ARADDR = araddr;
    assign M_AXI_ARVALID = arvalid;
    assign M_AXI_ARPROT = 3'b0;
    /////////////////////
    //Read data (R)//////
    /////////////////////
    assign M_AXI_RREADY = rready;
    //////////////////////
    //Write response (B)//
    //////////////////////   
    assign M_AXI_BREADY = bready;
    
    //response error signal//
    assign wrsp_error       =   bready & M_AXI_BVALID & M_AXI_BRESP[1];
    assign rrsp_error       =   rready & M_AXI_RVALID & M_AXI_RRESP[1];
    assign rd_mismatched    =   ((M_AXI_RVALID && rready) && (M_AXI_RDATA != rdata));
    ///////////////////////////////
    //Write address channel (AW)///
    ///////////////////////////////
    /*
    The purpose of AW is to request address and data information from SoC.
    This is single beat of data transferring (burst)
    
    Note: awvalid/wvalid be ticked at the same time (synchronized)
    AXI_VALID is held active high until being accepted by another one 
    (slave/ push signal)
    */
    always_ff @(posedge M_AXI_ACLK)
    begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            awvalid <= 1'b0;
        end
        else if(M_AXI_AWREADY && awvalid)
        begin
            awvalid <= 1'b0;
        end
        else if(wpush)
        begin
            awvalid <= 1'b1;
        end
    end
    ///////////////////////////////
    /////Write data channel (W)////
    ///////////////////////////////
    /*
    This channel is to push data into the bank
    WVALID and WREADY handshake
    WVALID/AWVALID is asserted at the same time 
    WVALID is dependent on AWVALID 
    */
    always_ff @(posedge M_AXI_ACLK)
    begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            wvalid <= 1'b0;
        end
        else if(M_AXI_WREADY && wvalid)
        begin
            wvalid <= 1'b0;
        end
        else if(wpush)
        begin
            wvalid <= 1'b1;
        end
        else 
            wvalid <= awvalid;
    end
    ///////////////////////////////
    ///Read address channel (AR)///
    ///////////////////////////////
    /*
    This channel is quite simular to Write address channel 
    ARVALID signal is controlled by rpop (pop_read) in order 
    to request to pop data out
    */
    always_ff @(posedge M_AXI_ACLK)
    begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            arvalid <= 1'b0;
        end
        else if(M_AXI_ARREADY && arvalid)
        begin
            arvalid <= 1'b0;
        end
        else if(rpop)
        begin
            arvalid <= 1'b1;
        end
    end
    ///////////////////////////////
    /////Read data channel (R)/////
    ///////////////////////////////
    /*
    This channel returns the data from read request 
    In this channel, data checker always allows data to pop 
    So we dont need to restrict conditions for popping data 
    (RREADY is held active except the initial condition is not satisfied)
    */
    always_ff @(M_AXI_ACLK)
    begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            rready <= 1'b0;
        end
        else
        begin
            rready <= 1'b1;
        end
    end
    ///////////////////////////////
    //////Write response (B)///////
    ///////////////////////////////
    /*
    This channel is to return feedbacks when write data to memory (RAM)
    BREADY is active when data from write channel is accepted by the slave
    (both address and data)
    Always accept write response 
    This channel guarantees that there is no access being launched until 
    the fist beat is done.
    */
    always_ff @(posedge M_AXI_ACLK)
    begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            bready <= 1'b0;
        end
        else
        begin
            bready <= 1'b1;
        end
    end
    
    ///////////////////////////////
    //         User logic      ////
    ///////////////////////////////    
    /*
    Init address/data pairs
    */
    //write address/data//
    always @(wr_id)
    begin
        wdata <= wr_id;
        awaddr <= BASE_ADDR + (wr_id-1)*4;
    end
    
    //read address/data//
    always @(rd_id)
    begin
        rdata <= rd_id;
        araddr <= BASE_ADDR + (rd_id-1)*4;
    end
    
    ///////////////////////
    //Main write controller
    ///////////////////////
    /*
     By only issuing one request at a time, the control logic is
     simplified.
     Request a new write if:
      - A command was not just submitted
      - AW and W channels are both idle
      - A new request was not requested last cycle
     */
     always_ff @(posedge M_AXI_ACLK)
     begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            wpush <= 1'b0;
            wr_id <= 0;
        end
        else if(~awvalid && ~wvalid && ~wpush && ~(C_NUM_COMMANDS == 3))
        begin
            wpush <= 1'b1;
            wr_id <= wr_id + 1;
        end
        else
        begin
            wpush <= 1'b0;
            wr_id <= wr_id;
        end
     end
     
     //////////////////////////
     //Check last write completion
     /*
     Check the last write is being commited and responsed
     */
     always_ff @(posedge M_AXI_ACLK)
     begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            wr_done <= 1'b0;
        end
        else if((C_NUM_COMMANDS == 3) && M_AXI_BVALID)
        begin
            wr_done <= 1'b1;
        end
     end
     
     //////////////////////
    //Main read controller
    //////////////////////
    /*
     Request a new read if:
      -A command was not just submitted
      -AR channel is idle
      -A new request was not requested last cycle
     */
     always_ff @(posedge M_AXI_ACLK)
     begin
        if(!(M_AXI_ARESETN && i_gpio) || wr_done == 0)
        begin
            rpop <= 1'b0;
            rd_id <= 0;
        end
        else if(~arvalid && ~rpop && ~(C_NUM_COMMANDS == 3))
        begin
            rpop <= 1'b1;
            rd_id <= rd_id + 1;
        end
        else
        begin
            rpop <= 1'b0;
            rd_id <= rd_id;
        end
     end 
     
     
     //////////////////////////
     //Check last read completion
     /*
     Check the last write is being commited and responsed
     */
     always_ff @(posedge M_AXI_ACLK)
     begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            rd_done <= 1'b0;
        end
        else if((C_NUM_COMMANDS == 3) && M_AXI_RVALID)
        begin
            rd_done <= 1'b1;
        end
     end    
     
     /////////////////////////////
     //Design error detection
     //////////////////////////////
     //check error flag//
     always_ff @(posedge M_AXI_ACLK)
     begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            error_tick <= 1'b0;
        end
        else if(wrsp_error || rrsp_error || rd_mismatched)
        begin
            error_tick <= 1'b1;
        end
        else
        begin
            error_tick <= error_tick;
        end
     end
     
     
     always_ff @(posedge M_AXI_ACLK)
     begin
        if(!(M_AXI_ARESETN && i_gpio))
        begin
            suc_fin <= 1'b0;
        end
        else if(wr_done && rd_done && ~error_tick)
        begin
            suc_fin <= 1'b1;
        end
        else
        begin
            suc_fin <= suc_fin;
        end
     end
endmodule