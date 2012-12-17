###
### COIN-OR Clp API Wrapper
###

module Clp


export
    # Types
    CoinBigIndex,
    CoinBigDouble,
    ClpModel,

    # Methods
    load_problem,
    load_quadratic_objective,
    read_mps,
    copy_in_integer_information,
    delete_integer_information,
    resize,
    delete_rows,
    add_rows,
    delete_columns,
    add_columns,
    chg_row_lower,
    chg_row_upper,
    chg_column_lower,
    chg_column_upper,
    chg_obj_coefficients,
    drop_names,
    copy_names,
    get_num_rows,
    get_num_cols,
    number_rows,
    number_cols,
    primal_tolerance,
    set_primal_tolerance,
    dual_tolerance,
    set_dual_tolerance,
    dual_objective_limit,
    set_dual_objective_limit,
    objective_offset,
    set_objective_offset,
    problem_name,
    set_problem_name,
    number_iterations,
    get_iteration_count,
    set_number_iterations,
    maximum_iterations,
    set_maximum_iterations,
    maximum_seconds,
    set_maximum_seconds,
    hit_maximum_iterations,
    status,
    set_problem_status,
    secondary_status,
    set_secondary_status,
    optimization_direction,
    set_optimization_direction,
    get_row_activity,
    get_col_solution,
    primal_column_solution,
    primal_row_solution,
    get_row_price,
    dual_row_solution,
    get_reduced_cost,
    dual_column_solution,
    get_row_lower,
    row_lower,
    get_row_upper,
    row_upper,
    get_obj_coefficients,
    objective,
    get_col_lower,
    column_lower,
    get_col_upper,
    column_upper,
    getNumElements,
    get_vector_starts,
    get_indices,
    get_constraint_matrix,
    getVectorLengths,
    get_elements,
    get_obj_value,
    objective_value,
    integer_information,
    infeasibility_ray,
    unbounded_ray,
    statusExists,
    status_array,
    copyin_status,
    get_column_status,
    get_row_status,
    set_column_status,
    set_row_status,
    get_user_pointer,
    set_user_pointer,
    clear_call_back,
    log_level,
    set_log_level,
    length_names,
    row_name,
    column_name,
    initial_solve,
    initial_dual_solve,
    initial_primal_solve,
    initial_barrier_solve,
    initial_barrier_no_cross_solve,
    dual,
    primal,
    idiot,
    scaling,
    scaling_flag,
    crash,
    primal_feasible,
    dual_feasible,
    dual_bound,
    set_dual_bound,
    infeasibility_cost,
    set_infeasibility_cost,
    perturbation,
    set_perturbation,
    algorithm,
    set_algorithm,
    sum_dual_infeasibilities,
    number_dual_infeasibilities,
    sum_primal_infeasibilities,
    number_primal_infeasibilities,
    save_model,
    restore_model,
    check_solution,
    is_abandoned,
    is_proven_optimal,
    is_proven_primal_infeasible,
    is_proven_dual_infeasible,
    is_primal_objective_limit_reached,
    is_dual_objective_limit_reached,
    is_iteration_limit_reached,
    get_obj_sense,
    set_obj_sense

import Base.pointer


## Shared library interface setup
#{{{
const _jl_libClp = "libClp"

macro clp_ccall(func, args...)
    f = "Clp_$(func)"
    quote
        ccall(($f,_jl_libClp), $(args...))
    end
end

# Note: we assume COIN_BIG_INDEX and COIN_BIG_DOUBLE
# were not defined when compiling Clp (which is the
# default)
typealias CoinBigIndex Int32
typealias CoinBigDouble Float64

#}}}

## Main types definitions
#{{{
type ClpModel
    p::Ptr{Void}
    function ClpModel()
        p = @clp_ccall newModel Ptr{Void} ()
        prob = new(p)
        finalizer(prob, delete_model)
        return prob
    end
end

function delete_model(model::ClpModel)
    if model.p == C_NULL
        return
    end
    @clp_ccall deleteModel Void (Ptr{Void},) model.p
    model.p = C_NULL
    return
end

pointer(model::ClpModel) = model.p
#}}}

## Check functions for internal use
#{{{
# Functions which perform all sorts of
# sanity checks on input parameters and
# throw exceptions in case of errors.
# Ideally, it should never be possible
# to pass an invalid parameter to the
# underlying Clp API.

function _jl__check_model(model::ClpModel)
    if model.p == C_NULL
        error("Invalid ClpModel")
    end
    return true
end

function _jl__check_row_is_valid(model::ClpModel, row::Integer)
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) model.p
    if !(1 <= row <= num_rows)
        error("Invalid row $row (must be 1 <= row <= $num_rows)")
    end
    return true
end

function _jl__check_col_is_valid(model::ClpModel, col::Integer)
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    if !(1 <= col <= num_cols)
        error("Invalid col $col (must be 1 <= col <= $num_cols)")
    end
    return true
end

function _jl__check_file_is_readable(filename::String)
    try
        f = open(filename, "r")
        close(f)
    catch err
        error("file $filename not readable")
    end
    return true
end
#}}}

## CLP functions
#{{{

# Load model - loads some stuff and initializes others
# Loads a problem (the constraints on the
# rows are given by lower and upper bounds). If a pointer is NULL then the
# following values are the default:
#
# col_ub: all columns have upper bound infinity
# col_lb: all columns have lower bound 0
# row_ub: all rows have upper bound infinity
# row_lb: all rows have lower bound -infinity
# obj: all variables have 0 objective coefficient

# Just like the other load_problem() method except that the matrix is
# given in a standard column major ordered format (without gaps).
function load_problem (model::ClpModel,  num_cols::Integer, num_rows::Integer,
        start::Vector{CoinBigIndex}, index::Vector{Int32},
        value::Vector{Float64},
        col_lb::Vector{Float64}, col_ub::Vector{Float64},
        obj::Vector{Float64},
        row_lb::Vector{Float64}, row_ub::Vector{Float64})
    _jl__check_model(model)
    # TODO: allow empty arguments according to documentation
    @clp_ccall loadProblem Void (Ptr{Void},Int32,Int32,Ptr{CoinBigIndex},Ptr{Int32},
    Ptr{Float64},Ptr{Float64},Ptr{Float64},Ptr{Float64},Ptr{Float64},Ptr{Float64}) model.p num_cols num_rows start index value col_lb col_ub obj row_lb row_ub
    return
end

function load_problem (model::ClpModel,  constraint_matrix::SparseMatrixCSC{Float64,Int32}, 
    col_lb::Vector{Float64}, col_ub::Vector{Float64}, 
    obj::Vector{Float64}, row_lb::Vector{Float64},
    row_ub::Vector{Float64})
    # We need to convert to zero-based, but
    # TODO: don't make extra copies of arrays
    load_problem(model,constraint_matrix.n, constraint_matrix.m,constraint_matrix.colptr-int32(1),
        constraint_matrix.rowval-int32(1),constraint_matrix.nzval,
        col_lb,col_ub,obj,row_lb,row_ub)
end

# Read quadratic part of the objective (the matrix part).
function load_quadratic_objective(model::ClpModel,
        num_cols::Integer, start::Vector{CoinBigIndex},
        col::Int, element::Float64)
    # TODO
    error("TODO")
    return
end

# Read an mps file from the given filename.
function read_mps(model::ClpModel, mpsfile::String, keep_names::Bool, ignore_errors::Bool)
    _jl__check_model(model)
    _jl__check_file_is_readable(mpsfile)

    status = @clp_ccall readMps Int32 (Ptr{Void}, Ptr{Uint8}, Int32, Int32) model.p bytestring(mpsfile) keep_names ignore_errors
    if status != 0
        error("read_mps: error reading file $mpsfile")
    end
    return true
end
read_mps(model::ClpModel, mpsfile::String) = read_mps(model, mpsfile, true, false)

# Copy in integer information.
function copy_in_integer_information(model::ClpModel, information::Vector{Uint8})
    # TODO
    error("TODO")
    return
end

# Drop integer information.
function delete_integer_information(model::ClpModel)
    # TODO
    error("TODO")
    return
end

# Resize rim part of model.
function resize (model::ClpModel, new_num_rows::Int32, new_num_cols::Int32)
    # TODO
    error("TODO")
    return
end

# Delete rows.
function delete_rows(model::ClpModel, number::Int32, which::Vector{Int32})
    # TODO
    error("TODO")
    return
end
delete_rows(model::ClpModel, which::Vector{Int32}) = delete_rows(model, length(which), which)

# Add rows.
function add_rows(model::ClpModel, number::Integer, row_lower::Vector{Float64},
        row_upper::Vector{Float64},
        row_starts::Vector{Int32}, columns::Vector{Int32},
        elements::Vector{Float64})
    _jl__check_model(model)

    @clp_ccall addRows Void (Ptr{Void}, Int32, Ptr{Float64}, Ptr{Float64}, Ptr{Int32}, Ptr{Int32}, Ptr{Float64}) model.p number row_lower row_upper row_starts columns elements
end

# Delete columns.
function delete_columns(model::ClpModel, number::Int32, which::Vector{Int32})
    # TODO
    error("TODO")
    return
end

# Add columns.
function add_columns(model::ClpModel, number::Integer, column_lower::Vector{Float64},
        column_upper::Vector{Float64},
        objective::Vector{Float64},
        column_starts::Vector{Int32}, rows::Vector{Int32},
        elements::Vector{Float64})
    _jl__check_model(model)

    @clp_ccall addColumns Void (Ptr{Void}, Int32, Ptr{Float64}, Ptr{Float64}, Ptr{Float64}, Ptr{Int32}, Ptr{Int32}, Ptr{Float64}) model.p number column_lower column_upper objective column_starts rows elements
    
end

function add_columns(model::ClpModel, column_lower::Vector{Float64},
        column_upper::Vector{Float64},
        objective::Vector{Float64},
        new_columns::SparseMatrixCSC{Float64,Int32})

    add_columns(model, new_columns.n, column_lower, column_upper, objective, new_columns.colptr-int32(1), new_columns.rowval-int32(1), new_columns.nzval)
end


# Change row lower bounds.
function chg_row_lower(model::ClpModel, row_lower::Vector{Float64})
    _jl__check_model(model)
    if (length(row_lower) != get_num_rows(model))
        error("Array length must match number of rows in the model")
    end

    @clp_ccall chgRowLower Void (Ptr{Void}, Ptr{Float64}) model.p row_lower
end

# Change row upper bounds.
function chg_row_upper(model::ClpModel, row_upper::Vector{Float64})
    _jl__check_model(model)
    if (length(row_upper) != get_num_rows(model))
        error("Array length must match number of rows in the model")
    end
    
    @clp_ccall chgRowUpper Void (Ptr{Void}, Ptr{Float64}) model.p row_upper
end

# Change column lower bounds.
function chg_column_lower(model::ClpModel, column_lower::Vector{Float64})
    _jl__check_model(model)
    if (length(column_lower) != get_num_cols(model))
        error("Array length must match number of columns in the model")
    end

    @clp_ccall chgColumnLower Void (Ptr{Void}, Ptr{Float64}) model.p column_lower 
end

# Change column upper bounds.
function chg_column_upper(model::ClpModel, column_upper::Vector{Float64})
    _jl__check_model(model)
    if (length(column_lower) != get_num_cols(model))
        error("Array length must match number of columns in the model")
    end

    @clp_ccall chgColumnUpper Void (Ptr{Void}, Ptr{Float64}) model.p column_upper 
end

# Change objective coefficients.
function chg_obj_coefficients(model::ClpModel, obj_in::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Drops names - makes lengthnames 0 and names empty.
function drop_names(model::ClpModel)
    # TODO
    error("TODO")
    return
end

# Copy in names.
function copy_names(model::ClpModel, row_names::Vector{Vector{Uint8}},
        columnNames::Vector{Vector{Uint8}})
    # TODO
    error("TODO")
    return
end

# Number of rows.
function get_num_rows(model::ClpModel)
    _jl__check_model(model)
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) model.p
end
number_rows = get_num_rows

# Number of columns.
function get_num_cols(model::ClpModel)
    _jl__check_model(model)
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
end
number_cols = get_num_cols

# Get primal tolerance.
function primal_tolerance(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall primalTolerance Float64 (Ptr{Void},) model.p
end

# Set primal tolerance to use.
function set_primal_tolerance(model::ClpModel, value::Float64)
    _jl__check_model(model)
    @clp_ccall setPrimalTolerance Void (Ptr{Void}, Float64) model.p value
    return
end

# Get dual tolerance.
function dual_tolerance(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall dualTolerance Float64 (Ptr{Void},) model.p
end

# Set dual tolerance to use.
function set_dual_tolerance(model::ClpModel, value::Float64)
    _jl__check_model(model)
    @clp_ccall setDualTolerance Void (Ptr{Void}, Float64) model.p value
    return
end

# Get dual objective limit.
function dual_objective_limit(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall dualObjectiveLimit Float64 (Ptr{Void},) model.p
end

# Set dual objective limit.
function set_dual_objective_limit(model::ClpModel, value::Float64)
    @clp_ccall setDualObjectiveLimit Void (Ptr{Void}, Float64) model.p value
    return
end

# Get objective offset.
function objective_offset(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall dualObjectiveLimit Float64 (Ptr{Void},) model.p
end

# Set objective offset.
function set_objective_offset(model::ClpModel, value::Float64)
    @clp_ccall setObjectiveOffset Void (Ptr{Void}, Float64) model.p value
    return
end

# Fill in array with problem name.
function problem_name(model::ClpModel)
    _jl__check_model(model)
    problem_name_p = pointer(Array(Uint8, 1000))
    @clp_ccall problemName Void (Ptr{Void}, Int32, Ptr{Uint8}) model.p 1000 problem_name_p
    return string(problem_name_p)
end

# Set problem name.  Must have \0 at end.
function set_problem_name(model::ClpModel, max_number_chars::Int32, array::Vector{Uint8})
    # TODO
    error("TODO")
    return intvalue
end
set_problem_name(model::ClpModel, array::Vector{Uint8}) = set_problem_name(model, length(array)+1, array)

# Get number of iterations
function number_iterations(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall numberIterations Int32 (Ptr{Void},) model.p
end
get_iteration_count = number_iterations

# Set number of iterations
function set_number_iterations(model::ClpModel, iters::Integer)
    _jl__check_model(model)
    @clp_ccall setNumberIterations Void (Ptr{Void}, Int32) model.p iters
    return
end

# Get maximum number of iterations
function maximum_iterations(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall maximumIterations Int32 (Ptr{Void},) model.p
end

# Set maximum number of iterations
function set_maximum_iterations(model::ClpModel, max_iters::Integer)
    _jl__check_model(model)
    @clp_ccall setMaximumIterations Void (Ptr{Void}, Int32) model.p max_iters
    return
end

# Get maximum time in seconds (from when set is called)
function maximum_seconds(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall maximumSeconds Float64 (Ptr{Void},) model.p
end

# Set maximum time in seconds (from when set is called)
function set_maximum_seconds(model::ClpModel, max_secs::Real)
    _jl__check_model(model)
    @clp_ccall setMaximumSeconds Void (Ptr{Void}, Float64) model.p max_secs
    return
end

# Query whether maximum iterations or time were hit
function hit_maximum_iterations(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall hitMaximumIterations Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Get the status of the problem:
#   0 - optimal
#   1 - primal infeasible
#   2 - dual infeasible
#   3 - stopped on iterations etc
#   4 - stopped due to errors
function status(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall status Int32 (Ptr{Void},) model.p
end

# Set the status of the problem.
function set_problem_status(model::ClpModel, status::Integer)
    _jl__check_model(model)
    @clp_ccall setProblemStatus Void (Ptr{Void}, Int32) model.p status
    return
end

# Get the secondary status of problem - may get extended:
#   0 - none
#   1 - primal infeasible because dual limit reached
#   2 - scaled problem optimal - unscaled has primal infeasibilities
#   3 - scaled problem optimal - unscaled has dual infeasibilities
#   4 - scaled problem optimal - unscaled has both dual and primal infeasibilities
function secondary_status(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall secondaryStatus Int32 (Ptr{Void},) model.p
end

# Set the secondary status of the problem.
function set_secondary_status(model::ClpModel, status::Integer)
    _jl__check_model(model)
    @clp_ccall setSecondaryStatus Void (Ptr{Void}, Int32) model.p status
    return
end

# Get the direction of optimization:
#   1 - minimize
#   -1 - maximize
#   0 - ignore
# XXX: for some reason this returns a floating point (???)
function optimization_direction(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall optimizationDirection Float64 (Ptr{Void},) model.p
end

# Set the direction of optimization.
# XXX: for some reason this takes a floating point argument (???)
function set_optimization_direction(model::ClpModel, value::Real)
    _jl__check_model(model)
    @clp_ccall setOptimizationDirection Void (Ptr{Void}, Float64) model.p value
    return
end

# Get primal row solution.
function get_row_activity(model::ClpModel)
    _jl__check_model(model)
    row_activity_p = @clp_ccall getRowActivity Ptr{Float64} (Ptr{Void},) model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) model.p
    row_activity = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_activity), row_activity_p, num_rows * sizeof(Float64))
    return row_activity
end
primal_row_solution = get_row_activity

# Get primal column solution.
function get_col_solution(model::ClpModel)
    _jl__check_model(model)
    col_solution_p = @clp_ccall getColSolution Ptr{Float64} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    col_solution = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(col_solution), col_solution_p, num_cols * sizeof(Float64))
    return col_solution
end
primal_column_solution = get_col_solution

# Get dual row solution.
function get_row_price(model::ClpModel)
    _jl__check_model(model)
    row_price_p = @clp_ccall getRowPrice Ptr{Float64} (Ptr{Void},) model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) model.p
    row_price = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_price), row_price_p, num_rows * sizeof(Float64))
    return row_price
end
dual_row_solution = get_row_price

# Get dual column solution (i.e. the reduced costs).
function get_reduced_cost(model::ClpModel)
    _jl__check_model(model)
    reduced_cost_p = @clp_ccall getReducedCost Ptr{Float64} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    reduced_cost = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(reduced_cost), reduced_cost_p, num_cols * sizeof(Float64))
    return reduced_cost
end
dual_column_solution = get_reduced_cost

# Get row lower bounds.
function get_row_lower(model::ClpModel)
    _jl__check_model(model)
    row_lower_p = @clp_ccall getRowLower Ptr{Float64} (Ptr{Void},) model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) model.p
    row_lower = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_lower), row_lower_p, num_rows * sizeof(Float64))
    return row_lower
end
row_lower = get_row_lower

# Get row upper bounds.
function get_row_upper(model::ClpModel)
    _jl__check_model(model)
    row_upper_p = @clp_ccall getRowUpper Ptr{Float64} (Ptr{Void},) model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) model.p
    row_upper = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_upper), row_upper_p, num_rows * sizeof(Float64))
    return row_upper
end
row_upper = get_row_upper

# Get objective coefficients.
function get_obj_coefficients(model::ClpModel)
    _jl__check_model(model)
    obj_coefficients_p = @clp_ccall getObjCoefficients Ptr{Float64} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    obj_coefficients = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(obj_coefficients), obj_coefficients_p, num_cols * sizeof(Float64))
    return obj_coefficients
end
objective = get_obj_coefficients

# Get column lower bounds.
function get_col_lower(model::ClpModel)
    _jl__check_model(model)
    col_lower_p = @clp_ccall getColLower Ptr{Float64} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    col_lower = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(col_lower), col_lower_p, num_cols * sizeof(Float64))
    return col_lower
end
column_lower = get_col_lower

# Get column upper bounds.
function get_col_upper(model::ClpModel)
    _jl__check_model(model)
    col_upper_p = @clp_ccall getColUpper Ptr{Float64} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    col_upper = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(col_upper), col_upper_p, num_cols * sizeof(Float64))
    return col_upper
end
column_upper = get_col_upper

# Get the number of elements in matrix.
function getNumElements(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall getNumElements Int32 (Ptr{Void},) model.p
end

# Get column starts in matrix.
function get_vector_starts(model::ClpModel)
    _jl__check_model(model)
    vec_starts_p = @clp_ccall getVectorStarts Ptr{CoinBigIndex} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    vec_starts = Array(CoinBigIndex, num_cols+1)
    ccall(:memcpy, Ptr{Void}, (Ptr{CoinBigIndex}, Ptr{CoinBigIndex}, Uint), pointer(vec_starts), vec_starts_p, (num_cols+1) * sizeof(CoinBigIndex))
    return vec_starts
end

# Get row indices in matrix.
function get_indices(model::ClpModel)
    _jl__check_model(model)
    # getIndices returns an "int*", how do we know it's Int32??
    row_indices_p = @clp_ccall getIndices Ptr{Int32} (Ptr{Void},) model.p
    num_elts = getNumElements(model)
    row_indices = Array(Int32, num_elts)
    ccall(:memcpy, Ptr{Void}, (Ptr{Int32}, Ptr{Int32}, Uint), pointer(row_indices), row_indices_p, num_elts * sizeof(Int32))
    return row_indices
end

function get_constraint_matrix(model::ClpModel)
    _jl__check_model(model)
    @assert CoinBigIndex == Int32
    # SparseMatrixCSC requires same integer type for colptr and rowval
    num_cols = convert(Int,get_num_cols(model))
    num_rows = convert(Int,get_num_rows(model))
    colptr = get_vector_starts(model) + int32(1)
    rowval = get_indices(model) + convert(CoinBigIndex,1)
    nzval = get_elements(model)

    return SparseMatrixCSC{Float64,CoinBigIndex}(num_rows,num_cols,colptr,rowval,nzval)

end

# Get column vector lengths in matrix.
function getVectorLengths(model::ClpModel)
    _jl__check_model(model)
    vec_len_p = @clp_ccall getVectorLengths Ptr{Int32} (Ptr{Void},) model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) model.p
    vec_len = Array(Int32, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Int32}, Ptr{Int32}, Uint), pointer(vec_len), vec_len_p, num_cols * sizeof(Int32))
    return vec_len
end

# Get element values in matrix.
function get_elements(model::ClpModel)
    _jl__check_model(model)
    elements_p = @clp_ccall getElements Ptr{Float64} (Ptr{Void},) model.p
    num_elts = getNumElements(model)
    elements = Array(Float64, num_elts)
    ccall(:memcpy, Ptr{Void}, (Ptr{Int32}, Ptr{Int32}, Uint), pointer(elements), elements_p, num_elts * sizeof(Float64))
    return elements
end

# Get objective value.
function get_obj_value(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall getObjValue Float64 (Ptr{Void},) model.p
end
objective_value = get_obj_value

# Get integer information.
function integer_information(model::ClpModel)
    # TODO
    error("TODO")
    return # vector of chars (?)
end

# Get an infeasibility ray (nothing returned if none/wrong).
function infeasibility_ray(model::ClpModel)
    # TODO
    error("TODO")
    # XXX free the array returned by the ccall!
    return # array of Float64 or nothing
end

# Get an unbounded ray (nothing returned if none/wrong).
function unbounded_ray(model::ClpModel)
    # TODO
    error("TODO")
    # XXX free the array returned by the ccall!
    return # array of Float64 or nothing
end

# Query whether the status array exists (partly for OsiClp).
function statusExists(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall statusExists Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Return the address of the status array (char[num_rows+num_cols]).
# Status values are:
#   0 - free
#   1 - basic
#   2 - at upper
#   3 - at lower
#   4 - superbasic
#   (5 - fixed)
#   [???]
function status_array(model::ClpModel)
    # TODO
    error("TODO")
    return # pointer to unsigned chars (?)
end

# Copy in the status vector
# XXX copyin rather than copy_in, WTF?
# XXX input vector of unsigned chars (?)
function copyin_status(model::ClpModel, status_array::Vector{Uint8})
    # TODO
    error("TODO")
    return
end

# Get variable basis info.
# Note that we adjust from one-based indices
function get_column_status(model::ClpModel, sequence::Integer)
    _jl__check_model(model)
    _jl__check_col_is_valid(model, sequence)
    return @clp_ccall getColumnStatus Int32 (Ptr{Void},Int32) model.p (sequence-1)
end

# Get row basis info.
function get_row_status(model::ClpModel, sequence::Integer)
    _jl__check_model(model)
    _jl__check_row_is_valid(model, sequence)
    return @clp_ccall getRowStatus Int32 (Ptr{Void},Int32) model.p (sequence-1)
end

# Set variable basis info (and value if at bound).
function set_column_status(model::ClpModel, sequence::Integer, value::Integer)
    _jl__check_model(model)
    _jl__check_col_is_valid(model, sequence)
    @clp_ccall setColumnStatus Int32 (Ptr{Void},Int32,Int32) model.p (sequence-1) value
end

# Set row basis info (and value if at bound).
function set_row_status(model::ClpModel, sequence::Integer, value::Integer)
    _jl__check_model(model)
    _jl__check_row_is_valid(model, sequence)
    @clp_ccall setRowStatus Int32 (Ptr{Void},Int32,Int32) model.p (sequence-1) value
end

# Set user pointer (used for generic purposes)
function get_user_pointer(model::ClpModel)
    # TODO
    error("TODO")
    return # Ptr{Void}
end

# Get user pointer (used for generic purposes)
function set_user_pointer(model::ClpModel, pointer::Ptr{Void})
    # TODO
    error("TODO")
    return
end

# Pass in callback function.
# Message numbers up to 1000000 are Clp, Coin ones have 1000000 added
# TODO
# COINLIBAPI void COINLINKAGE Clp_registerCallBack(Clp_Simplex * model,
#        callback userCallBack);

# Unset callback function.
function clear_call_back(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall clearCallBack Void (Ptr{Void},) model.p
    return
end

# Get amount of print out:
#   0 - none
#   1 - just final
#   2 - just factorizations
#   3 - as 2 plus a bit more
#   4 - verbose
#   above that: 8,16,32 etc just for selective debug.
function log_level(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall logLevel Int32 (Ptr{Void},) model.p
end

# Set amount of print out.
function set_log_level(model::ClpModel, value::Integer)
    _jl__check_model(model)
    @clp_ccall setLogLevel Void (Ptr{Void}, Int32) model.p value
    return
end

# Get length of names (0 means no names).
function length_names(model::ClpModel)
    _jl__check_model(model)
    return @clp_ccall lengthNames Int32 (Ptr{Void},) model.p
end

# Return an array with a row name.
# XXX deviation from Clp API syntax, which fills a user-provided
# array of chars:
#   void Clp_rowName(Clp_Simplex * model, int iRow, char * name);
function row_name(model::ClpModel, row::Integer)
    _jl__check_model(model)
    _jl__check_row_is_valid(model, row)
    size = @clp_ccall lengthNames Int32 (Ptr{Void},) model.p
    row_name_p = pointer(Array(Uint8, size+1))
    @clp_ccall rowName Void (Ptr{Void}, Int32, Ptr{Uint8}) model.p (row-1) row_name_p
    row_name = bytestring(row_name_p)
    return row_name
end

# Return an array with a column name.
# XXX deviation from Clp API syntax, which fills a user-provided
# array of chars:
#   void Clp_columnName(Clp_Simplex * model, int iColumn, char * name);
function column_name(model::ClpModel, col::Integer)
    _jl__check_model(model)
    _jl__check_col_is_valid(model, col)
    size = @clp_ccall lengthNames Int32 (Ptr{Void},) model.p
    col_name_p = pointer(Array(Uint8,size+1))
    @clp_ccall columnName Void (Ptr{Void}, Int32, Ptr{Uint8}) model.p (col-1) col_name_p
    col_name = bytestring(col_name_p)
    return col_name
end

# General solve algorithm which can do presolve.
function initial_solve(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall initialSolve Int32 (Ptr{Void},) model.p
end

# Dual initial solve.
function initial_dual_solve(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall initialDualSolve Int32 (Ptr{Void},) model.p
end

# Primal initial solve.
function initial_primal_solve(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall initialPrimalSolve Int32 (Ptr{Void},) model.p
end

# Barrier initial solve.
function initial_barrier_solve(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall initialBarrierSolve Int32 (Ptr{Void},) model.p
end

# Barrier initial solve, no crossover.
function initial_barrier_no_cross_solve(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall initialBarrierNoCrossSolve Int32 (Ptr{Void},) model.p
end

# Dual algorithm.
function dual(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall dual Int32 (Ptr{Void},) model.p
end

# Primal algorithm.
function primal(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall primal Int32 (Ptr{Void},) model.p
end

# Solve the problem with the idiot code.
function idiot(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall idiot Int32 (Ptr{Void},) model.p
end

# Set or unset scaling:
#  0 - off
#  1 - equilibrium
#  2 - geometric
#  3 - auto
#  4 - dynamic (later)
function scaling(model::ClpModel, mode::Integer)
    _jl__check_model(model)
    if !(0 <= mode <= 4)
        error("Invalid clp scaling mode $mode (must be between 0 and 4)")
    end
    @clp_ccall scaling Void (Ptr{Void}, Int32) model.p mode
    return
end

# Get scaling flag.
function scaling_flag(model::ClpModel)
    _jl__check_model(model)
    return @clp_ccall scalingFlag Int32 (Ptr{Void},) model.p
end

# Crash (at present just aimed at dual); returns:
#   -2 if dual preferred and crash basis created
#   -1 if dual preferred and all slack basis preferred
#   0 if basis going in was not all slack
#   1 if primal preferred and all slack basis preferred
#   2 if primal preferred and crash basis created.
#
#   If gap between bounds <="gap" variables can be flipped
#
#   If "pivot" is:
#     0 - No pivoting (so will just be choice of algorithm)
#     1 - Simple pivoting e.g. gub
#     2 - Mini iterations
function crash(model::ClpModel, gap::Float64, pivot::Int32)
    _jl__check_model(model)
    if !(0 <= pivot <= 2)
        error("Invalid clp crash pivot value $pivot (must be between 0 and 2)")
    end
    return @clp_ccall crash Int32 (Ptr{Void}, Float64, Int32) model.p gap pivot
end

# Query whether the problem is primal feasible.
function primal_feasible(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall primalFeasible Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether the problem is dual feasible.
function dual_feasible(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall dualFeasible Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Get dual bound.
function dual_bound(model::ClpModel)
    _jl__check_model(model)
    @clp_ccall dualBound Float64 (Ptr{Void},) model.p
end

# Set dual bound.
function set_dual_bound(model::ClpModel, value::Real);
    _jl__check_model(model)
    @clp_ccall setDualBound Void (Ptr{Void}, Float64) model.p value
    return
end

# Get infeasibility cost.
function infeasibility_cost(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall infeasibilityCost Float64 (Ptr{Void},) model.p
end

# Set infeasibility cost.
function set_infeasibility_cost(model::ClpModel, value::Real);
    _jl__check_model(model)
    @clp_ccall setInfeasibilityCost Void (Ptr{Void}, Float64) model.p value
    return
end

# Get Perturbation. Values are:
#   50  - switch on perturbation
#   100 - auto perturb if takes too long (1.0e-6 largest nonzero)
#   101 - we are perturbed
#   102 - don't try perturbing again
# The default is 100; others are for playing.
function perturbation(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall perturbation Int32 (Ptr{Void},) model.p
end

# Set Perturbation.
function set_perturbation(model::ClpModel, value::Integer);
    _jl__check_model(model)
    if !(value == 50 || value == 100 || value == 101 || value == 102)
        error("Invalid clp perturbation value: $value (must be one of 50,100,101,102)")
    end
    @clp_ccall setPerturbation Void (Ptr{Void}, Int32) model.p value
    return
end

# Get current (or last) algorithm.
function algorithm(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall algorithm Int32 (Ptr{Void},) model.p
end

# Set algorithm.
function set_algorithm(model::ClpModel, value::Integer);
    _jl__check_model(model)
    # XXX which values of the algorithm are valid ???
    @clp_ccall setAlgorithm Void (Ptr{Void}, Int32) model.p value
    return
end

# Get the sum of dual infeasibilities.
function sum_dual_infeasibilities(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall sumDualInfeasibilities Float64 (Ptr{Void},) model.p
end

# Get the number of dual infeasibilities.
function number_dual_infeasibilities(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall numberDualInfeasibilities Int32 (Ptr{Void},) model.p
end

# Get the sum of primal infeasibilities.
function sum_primal_infeasibilities(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall sumPrimalInfeasibilities Float64 (Ptr{Void},) model.p
end

# Get the number of primal infeasibilities.
function number_primal_infeasibilities(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall numberPrimalInfeasibilities Int32 (Ptr{Void},) model.p
end

# Save model to file, returns 0 if success.  This is designed for
# use outside algorithms so does not save iterating arrays etc.
# It does not save any messaging information.
# Does not save scaling values.
# It does not know about all types of virtual functions.
function save_model(model::ClpModel, file_name::Vector{Uint8});
    # todo
    error("todo")
    return # Int32 [Bool]
end

# Restore model from file, returns 0 if success,
# deletes current model.
function restore_model(model::ClpModel, file_name::Vector{Uint8});
    # todo
    error("todo")
    return # Int32 [Bool]
end

# Just check solution (for external use) - sets sum of
# infeasibilities etc.
function check_solution(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall checkSolution Void (Ptr{Void},) model.p
    return
end

# Query whether there are numerical difficulties.
function is_abandoned(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isAbandoned Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether optimality is proven.
function is_proven_optimal(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isProvenOptimal Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether primal infeasiblity is proven.
function is_proven_primal_infeasible(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isProvenPrimalInfeasible Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether dual infeasiblity is proven.
function is_proven_dual_infeasible(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isProvenDualInfeasible Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether the given primal objective limit is reached.
function is_primal_objective_limit_reached(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isPrimalObjectiveLimitReached Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether the given dual objective limit is reached.
function is_dual_objective_limit_reached(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isDualObjectiveLimitReached Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Query whether the iteration limit is reached
function is_iteration_limit_reached(model::ClpModel)
    _jl__check_model(model)
    ret = @clp_ccall isIterationLimitReached Int32 (Ptr{Void},) model.p
    return ret != 0
end

# Get the direction of optimization:
#   1  - minimize
#   -1 - maximize
#   0  - ignore
# XXX the return value is floating point, WTF???
function get_obj_sense(model::ClpModel);
    _jl__check_model(model)
    @clp_ccall getObjSense Float64 (Ptr{Void},) model.p
end

# Set the direction of optimization.
# XXX the value is floating point, WTF???
function set_obj_sense(model::ClpModel, objsen::Real);
    _jl__check_model(model)
    if !(objset == -1 || objset == 0 || objsen == 1)
        error("Invalid clp objsense $objsense (should be -1,0 or 1)")
    end
    @clp_ccall setObjSense Void (Ptr{Void}, Float64) model.p objsen
    return
end

#### XXX to be continued...

# Set primal column solution.
#COINLIBAPI void COINLINKAGE Clp_setColSolution(Clp_Simplex * model, const double * input);
# XXX TODO

#    /** Print model for debugging purposes */
#    COINLIBAPI void COINLINKAGE Clp_printModel(Clp_Simplex * model, const char * prefix);
#    /* Small element value - elements less than this set to zero,
#       default is 1.0e-20 */
#    COINLIBAPI double COINLINKAGE Clp_getSmallElementValue(Clp_Simplex * model);
#    COINLIBAPI void COINLINKAGE Clp_setSmallElementValue(Clp_Simplex * model, double value);

#}}}

end # module
