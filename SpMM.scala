package spmm

import chisel3._
import chisel3.util._

class SpMMIO extends Bundle {
  val start = Input(Bool())

  val inputReady = Output(Bool())
  val lhsRowIdx = Input(Vec(16, UInt(8.W)))
  val lhsCol = Input(Vec(16, UInt(8.W)))
  val lhsData = Input(Vec(16, F44()))

  val rhsReset = Input(Bool())
  val rhsData = Input(Vec(16, F44()))

  val outData = Output(Vec(16, F44()))
  val outValid = Output(Bool())
}

class SpMM extends Module {
    val io = IO(new SpMMIO)
    val issue = Module(new issue)
    issue.io.start := io.start
    io.inputReady := issue.io.inputReady
    issue.io.lhsRowIdx := io.lhsRowIdx
    issue.io.lhsCol := io.lhsCol
    issue.io.lhsData := io.lhsData
    issue.io.rhsReset := io.rhsReset
    issue.io.rhsData := io.rhsData
    io.outData := issue.io.outData
    io.outValid := issue.io.outValid
}

object Main {
    def main(args: Array[String]): Unit = {
        emitVerilog(new SpMM)
    }
}

class innerproduct extends Module {    
    val io = IO(new Bundle{
        val lhsCol = Input(Vec(16,UInt(8.W)))
        val lhsData = Input(Vec(16,F44()))
        val rhsData = Input(Vec(16,F44()))
        val outData = Output(Vec(16,F44()))
    })
    
    val product = Reg(Vec(16,F44()))
    
    for(i<-0 until 16){
        product(i) := io.lhsData(i) * io.rhsData(io.lhsCol(i)) 
    }

    io.outData := product
}

class reduction extends Module {    
    val io = IO(new Bundle{
        val product = Input(Vec(16,F44()))
        val border = Input(F44())
        val split = Input(Vec(17,Bool()))
        val target_row = Input(Vec(16,UInt(8.W)))
        val patial_product = Output(Vec(16,F44()))
        val output_border = Output(F44())
        val border_control = Input(Bool())
    })

    val tree_layer = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(16)(F44.zero)))))
    val tree_layer_reg = RegInit(VecInit(Seq.fill(4)(VecInit(Seq.fill(16)(F44.zero)))))
    val prefix = RegInit(VecInit(Seq.fill(17)(F44.zero)))
    val prefix_reg = RegInit(VecInit(Seq.fill(17)(F44.zero)))
    val output_reg = Reg(Vec(16,F44()))
    val output_border_reg = RegInit(F44.zero)
    val split_reverse = Wire(Vec(17,Bool()))
    val former_prefix = Wire(Vec(18,UInt(8.W)))
    val split_reg = Reg(Vec(17,Bool()))
    val target_row_reg = Reg(Vec(16,UInt(8.W)))
    
    split_reg := io.split
    target_row_reg := io.target_row
    io.patial_product := output_reg

    for(i<-0 until 17){
        split_reverse(i) := io.split(16-i) 
    }
    
    former_prefix(0) := 0.U
    
    for(i<-1 until 18){
        former_prefix(i) := i.U-1.U-PriorityEncoder(split_reverse.slice(17-i,17))
    }

    when(io.border_control){
        tree_layer(0)(0) := io.product(0) + F44.zero
    }
    .otherwise{
        tree_layer(0)(0) := io.border + io.product(0)
    }
    
    for(i<-1 until 16){
        tree_layer(0)(i) := tree_layer_reg(0)(i)
        tree_layer_reg(0)(i) := io.product(i)
    }
    
    for(i<-0 until 8){
        tree_layer(1)(2*i) := tree_layer_reg(1)(2*i)
        tree_layer_reg(1)(2*i) := tree_layer(0)(2*i)
        tree_layer(1)(2*i+1) := tree_layer(0)(2*i) + tree_layer(0)(2*i+1)
    }

    for(i<-0 until 4){
        tree_layer(2)(4*i) := tree_layer_reg(2)(4*i)
        tree_layer_reg(2)(4*i) := tree_layer(1)(4*i)
        tree_layer(2)(4*i+1) := tree_layer_reg(2)(4*i+1)
        tree_layer_reg(2)(4*i+1) := tree_layer(1)(4*i+1)
        tree_layer(2)(4*i+2) := tree_layer(1)(4*i+1) + tree_layer(1)(4*i+2)
        tree_layer(2)(4*i+3) := tree_layer(1)(4*i+1) + tree_layer(1)(4*i+3)
    }

    for(i<-0 until 2){
        for(j<-0 until 4){
            tree_layer(3)(8*i+j) := tree_layer_reg(3)(8*i+j)
            tree_layer_reg(3)(8*i+j) := tree_layer(2)(8*i+j)
            tree_layer(3)(8*i+4+j) := tree_layer(2)(8*i+3) + tree_layer(2)(8*i+4+j)
        }
    }

    for(i<-0 until 8){
        prefix(i+1) := prefix_reg(i+1)
        prefix_reg(i+1) := tree_layer(3)(i)
        prefix(9+i) := tree_layer(3)(7) + tree_layer(3)(8+i)
    }

    prefix(0) := F44.zero

    for(i<-1 until 17){
        when(split_reg(i)){
            output_reg(target_row_reg(i-1)) := prefix(i) - prefix(former_prefix(i))
        }
    }

    output_border_reg := prefix(16) - prefix(former_prefix(17))
    io.output_border := output_border_reg
}

class collect extends Module {
    val io = IO(new Bundle{
        val patial_product = Input(Vec(16,F44()))
        val mask = Input(Vec(16,Bool()))
        val row = Input(UInt(8.W))
        val write_control = Input(Bool())
        val result = Output(Vec(16,F44()))
        val reset_buffer = Input(Bool())
        val row_output = Input(UInt(8.W))
        val row_for_reset = Input(UInt(8.W))
    })

    val output_buffer = SyncReadMem(16,Vec(16,F44()))
    val result_reg = Reg(Vec(16,F44()))
    val zeros = RegInit(VecInit(Seq.fill(16)(F44.zero)))
    io.result := result_reg

    when(io.reset_buffer){
        output_buffer.write(io.row_for_reset,zeros)
    }
    
    when(io.write_control){
        output_buffer.write(io.row,io.patial_product,io.mask)
    }

    io.result := output_buffer.read(io.row_output,true.B)
}

class issue extends Module {
    val io = IO(new SpMMIO)

    val lhsData_buffer = SyncReadMem(16,Vec(16,F44()))
    val lhsCol_buffer = SyncReadMem(16,Vec(16,UInt(8.W)))
    val lhsRowIdx_buffer = Reg(Vec(16,UInt(8.W)))
    val rhs_buffer = SyncReadMem(16,Vec(16,F44()))
    val split = Reg(Vec(16,Vec(16,Bool())))
    val row_tag = Reg(Vec(16,Vec(16,UInt(8.W))))
    val lhsRow_count = Reg(UInt(8.W))
    val input_counter = Counter(16)
    val lhs_input_state = RegInit(false.B)
    val rhs_input_state = RegInit(false.B)
    val valid_reg_1 = RegInit(false.B)
    val inputReady_reg = RegInit(true.B)
    io.outValid := valid_reg_1
    io.inputReady := inputReady_reg & (~io.start)
    val innerproduct_row_counter = Counter(16)
    val innerproduct_col_counter = Counter(16)
    val output_for_innerproduct = Wire(Vec(16,F44()))
    val innerproduct_state = RegInit(false.B)

    val reduction_state = RegInit(false.B)
    val reduction_row_counter = Counter(17)
    val reduction_col_counter = Counter(16)
    val border_control_for_reduction = Reg(Bool())
    val output_for_reduction = Wire(Vec(16,F44()))

    val collect_reset_state = RegInit(false.B)
    val collect_reset_counter = Counter(16)
    val collect_write_row_counter = Counter(16)
    val collect_write_col_counter = Counter(16)
    val collect_write_state = RegInit(false.B)
    val mask_buffer = Reg(Vec(16,Vec(16,Bool())))
    val collect_output_row_counter = Counter(19)
    val collect_output_state = RegInit(false.B)
    val result_for_collect = Wire(Vec(16,F44()))

    val innerproduct = Module(new innerproduct)
    innerproduct.io.lhsCol := lhsCol_buffer.read(innerproduct_row_counter.value,true.B)
    innerproduct.io.lhsData := lhsData_buffer.read(innerproduct_row_counter.value,true.B)
    innerproduct.io.rhsData := rhs_buffer.read(innerproduct_col_counter.value,true.B)
    output_for_innerproduct := innerproduct.io.outData

    val reduction = Module(new reduction)
    reduction.io.product := output_for_innerproduct
    reduction.io.split(0) := true.B
    for(i<-0 until 16){
        reduction.io.split(i+1) := split(reduction_row_counter.value)(i)
    }
    reduction.io.target_row := row_tag(reduction_row_counter.value)
    reduction.io.border_control := border_control_for_reduction
    reduction.io.border := ShiftRegister(reduction.io.output_border,4)
    output_for_reduction := reduction.io.patial_product

    val collect = Module(new collect)
    collect.io.reset_buffer := collect_reset_state
    collect.io.row := collect_write_col_counter.value
    collect.io.write_control := collect_write_state
    collect.io.patial_product := output_for_reduction
    collect.io.mask := mask_buffer(collect_write_row_counter.value)
    collect.io.row_output := collect_output_row_counter.value(3,0)
    collect.io.row_for_reset := collect_reset_counter.value
    result_for_collect := collect.io.result
    io.outData := result_for_collect
    when(io.start){
        lhsRow_count := io.lhsRowIdx(15)(7,4)
        for(i<-0 until 16){
            for(j<-0 until 16){
                split(i)(j) := false.B
                mask_buffer(i)(j) := false.B
            }
        }
        inputReady_reg := false.B
        lhsRowIdx_buffer := io.lhsRowIdx
        lhsData_buffer.write(0.U,io.lhsData)
        lhsCol_buffer.write(0.U,io.lhsCol)
        when(io.rhsReset){
            rhs_buffer.write(0.U,io.rhsData)
        }

        input_counter.reset()
        lhs_input_state := true.B
        
        when(io.rhsReset){
            rhs_input_state := true.B
        }

        innerproduct_row_counter.reset()
        innerproduct_col_counter.reset()
        innerproduct_state := true.B
        collect_reset_counter.reset()
        collect_reset_state := true.B
    }
    
    when(lhs_input_state){
        lhsData_buffer.write((input_counter.value+1.U),io.lhsData)
        lhsCol_buffer.write((input_counter.value+1.U),io.lhsCol)
        when(input_counter.value===14.U){
            lhs_input_state := false.B
        }
        input_counter.inc()
    }

    when(rhs_input_state){
        rhs_buffer.write((input_counter.value+1.U),io.rhsData)
        when(input_counter.value===14.U){
            rhs_input_state := false.B
        }
    }

    when(innerproduct_state){
        when(innerproduct_col_counter.value===15.U){
            innerproduct_row_counter.inc()
            when(innerproduct_row_counter.value===lhsRow_count){
                innerproduct_state := false.B
            }
        }
        innerproduct_col_counter.inc()
        when(innerproduct_row_counter.value===0.U&&innerproduct_col_counter.value===12.U){
            reduction_state := true.B
            reduction_col_counter.reset()
            reduction_row_counter.reset()
            
        }
        when(innerproduct_row_counter.value===0.U&&innerproduct_col_counter.value===14.U){
            collect_write_state := true.B
            collect_write_row_counter.reset()
            collect_write_col_counter.reset()
        }
        when(innerproduct_row_counter.value===0.U&&innerproduct_col_counter.value===0.U){
            for(i<-15 until -1 by -1){
                split(lhsRowIdx_buffer(i)(7,4))(lhsRowIdx_buffer(i)(3,0)) := true.B
                row_tag(lhsRowIdx_buffer(i)(7,4))(lhsRowIdx_buffer(i)(3,0)) := i.U
            }
        }
        when(innerproduct_row_counter.value===0.U&&innerproduct_col_counter.value===1.U){
            for(i<-0 until 16){
                for(j<-0 until 16){
                    when(split(i)(j)){
                        mask_buffer(i)(row_tag(i)(j)) := true.B
                    }
                }
            }
            //border_control_for_reduction := true.B 
        }
        when(innerproduct_row_counter.value===0.U&&innerproduct_col_counter.value===3.U){
            border_control_for_reduction := true.B
        }
        when(innerproduct_row_counter.value===1.U&&innerproduct_col_counter.value===3.U){
            border_control_for_reduction := false.B
        }
    }

    when(reduction_state){
        when(reduction_col_counter.value===15.U){
            reduction_row_counter.inc()
        }
        when(reduction_row_counter.value===lhsRow_count+1.U&&reduction_col_counter.value===3.U){
            reduction_state := false.B
        }
        reduction_col_counter.inc()
    }

    when(collect_reset_state){
        collect_reset_counter.inc()
        when(collect_reset_counter.value===15.U){
            collect_reset_state := false.B
        }
    }

    when(collect_write_state){
        when(collect_write_col_counter.value===15.U){
            collect_write_row_counter.inc()
            when(collect_write_row_counter.value===lhsRow_count){
                collect_write_state := false.B
                inputReady_reg := true.B
            }
            when(collect_write_row_counter.value===0.U){
                collect_output_state := true.B
                collect_output_row_counter.reset()
            }
        }
        collect_write_col_counter.inc()
    }
    .otherwise{
        when(collect_output_state){
            collect_output_row_counter.inc()
            when(collect_output_row_counter.value===15.U){
                collect_output_state := false.B
            }
            valid_reg_1 := true.B
        }
        .otherwise{
            valid_reg_1 := false.B 
        } 
    }
}
