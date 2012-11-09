###
### COIN-OR Clp API Wrapper
###

## Shared library interface setup
#{{{
_jl_libClp = dlopen("libClp")

macro clp_ccall(func, args...)
    f = "Clp_$(func)"
    quote
        ccall(dlsym(_jl_libClp, $f), $args...)
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
        finalizer(prob, clp_delete_model)
        return prob
    end
end

function clp_delete_model(clp_model::ClpModel)
    if clp_model.p == C_NULL
        return
    end
    @clp_ccall deleteModel Void (Ptr{Void},) clp_model.p
    clp_model.p = C_NULL
    return
end

pointer(clp_model::ClpModel) = clp_model.p
#}}}

## Check functions for internal use
#{{{
# Functions which perform all sorts of
# sanity checks on input parameters and
# throw exceptions in case of errors.
# Ideally, it should never be possible
# to pass an invalid parameter to the
# underlying Clp API.

function _jl_clp__check_clp_model(clp_model::ClpModel)
    if clp_model.p == C_NULL
        error("Invalid GLPProb")
    end
    return true
end

function _jl_clp__check_row_is_valid(clp_model::ClpModel, row::Integer)
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
    if !(1 <= row <= num_rows)
        error("Invalid row $row (must be 1 <= row <= $num_rows)")
    end
    return true
end

function _jl_clp__check_col_is_valid(clp_model::ClpModel, col::Integer)
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    if !(1 <= col <= num_cols)
        error("Invalid col $col (must be 1 <= col <= $num_cols)")
    end
    return true
end

function _jl_clp__check_file_is_readable(filename::String)
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
function clp_load_problem (clp_model::ClpModel,  num_cols::Integer, num_rows::Integer,
        start::Vector{CoinBigIndex}, index::Vector{Int},
        value::Vector{Float64},
        col_lb::Vector{Float64}, col_ub::Vector{Float64},
        obj::Vector{Float64},
        row_lb::Vector{Float64}, row_ub::Vector{Float64})
    # todo
    error("todo")
    return
end

# Read quadratic part of the objective (the matrix part).
function clp_load_quadratic_objective(clp_model::ClpModel,
        num_cols::Integer, start::Vector{CoinBigIndex},
        col::Int, element::Float64)
    # TODO
    error("TODO")
    return
end

# Read an mps file from the given filename.
function clp_read_mps(clp_model::ClpModel, mpsfile::String, keep_names::Bool, ignore_errors::Bool)
    _jl_clp__check_clp_model(clp_model)
    _jl_clp__check_file_is_readable(mpsfile)

    status = @clp_ccall readMps Int32 (Ptr{Void}, Ptr{Uint8}, Int32, Int32) clp_model.p cstring(mpsfile) keep_names ignore_errors
    if status != 0
        error("clp_read_mps: error reading file $mpsfile")
    end
    return true
end
clp_read_mps(clp_model::ClpModel, mpsfile::String) = clp_read_mps(clp_model, mpsfile, true, false)

# Copy in integer information.
function clp_copy_in_integer_information(clp_model::ClpModel, information::Vector{Uint8})
    # TODO
    error("TODO")
    return
end

# Drop integer information.
function clp_delete_integer_information(clp_model::ClpModel)
    # TODO
    error("TODO")
    return
end

# Resize rim part of model.
function clp_resize (clp_model::ClpModel, new_num_rows::Int32, new_num_cols::Int32)
    # TODO
    error("TODO")
    return
end

# Delete rows.
function clp_delete_rows(clp_model::ClpModel, number::Int32, which::Vector{Int32})
    # TODO
    error("TODO")
    return
end
clp_delete_rows(clp_model::ClpModel, which::Vector{Int32}) = clp_delete_rows(clp_model, length(which), which)

# Add rows.
function clp_add_rows(clp_model::ClpModel, number::Int32, row_lower::Vector{Float64},
        row_upper::Vector{Float64},
        row_starts::Vector{Int32}, columns::Vector{Int32},
        elements::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Delete columns.
function clp_delete_columns(clp_model::ClpModel, number::Int32, which::Vector{Int32})
    # TODO
    error("TODO")
    return
end

# Add columns.
function clp_add_columns(clp_model::ClpModel, number::Int32, column_lower::Vector{Float64},
        column_upper::Vector{Float64},
        objective::Vector{Float64},
        column_starts::Vector{Int32}, rows::Vector{Int32},
        elements::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Change row lower bounds.
function clp_chg_row_lower(clp_model::ClpModel, row_lower::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Change row upper bounds.
function clp_chg_row_upper(clp_model::ClpModel, row_upper::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Change column lower bounds.
function clp_chg_column_lower(clp_model::ClpModel, column_lower::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Change column upper bounds.
function clp_chg_column_upper(clp_model::ClpModel, column_upper::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Change objective coefficients.
function clp_chg_obj_coefficients(clp_model::ClpModel, obj_in::Vector{Float64})
    # TODO
    error("TODO")
    return
end

# Drops names - makes lengthnames 0 and names empty.
function clp_drop_names(clp_model::ClpModel)
    # TODO
    error("TODO")
    return
end

# Copy in names.
function clp_copy_names(clp_model::ClpModel, row_names::Vector{Vector{Uint8}},
        columnNames::Vector{Vector{Uint8}})
    # TODO
    error("TODO")
    return
end

# Number of rows.
function clp_get_num_rows(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
end
clp_number_rows = clp_get_num_rows

# Number of columns.
function clp_get_num_cols(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
end
clp_number_cols = clp_get_num_cols

# Get primal tolerance.
function clp_primal_tolerance(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall primalTolerance Float64 (Ptr{Void},) clp_model.p
end

# Set primal tolerance to use.
function clp_set_primal_tolerance(clp_model::ClpModel, value::Float64)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setPrimalTolerance Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Get dual tolerance.
function clp_dual_tolerance(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall dualTolerance Float64 (Ptr{Void},) clp_model.p
end

# Set dual tolerance to use.
function clp_set_dual_tolerance(clp_model::ClpModel, value::Float64)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setDualTolerance Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Get dual objective limit.
function clp_dual_objective_limit(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall dualObjectiveLimit Float64 (Ptr{Void},) clp_model.p
end

# Set dual objective limit.
function clp_set_dual_objective_limit(clp_model::ClpModel, value::Float64)
    @clp_ccall setDualObjectiveLimit Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Get objective offset.
function clp_objective_offset(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall dualObjectiveLimit Float64 (Ptr{Void},) clp_model.p
end

# Set objective offset.
function clp_set_objective_offset(clp_model::ClpModel, value::Float64)
    @clp_ccall setObjectiveOffset Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Fill in array with problem name.
function clp_problem_name(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    problem_name_p = pointer(Array(Uint8, 1000))
    @clp_ccall problemName Void (Ptr{Void}, Int32, Ptr{Uint8}) clp_model.p 1000 problem_name_p
    return string(problem_name_p)
end

# Set problem name.  Must have \0 at end.
function clp_set_problem_name(clp_model::ClpModel, max_number_chars::Int32, array::Vector{Uint8})
    # TODO
    error("TODO")
    return intvalue
end
clp_set_problem_name(clp_model::ClpModel, array::Vector{Uint8}) = clp_set_problem_name(clp_model, length(array)+1, array)

# Get number of iterations
function clp_number_iterations(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall numberIterations Int32 (Ptr{Void},) clp_model.p
end
clp_get_iteration_count = clp_number_iterations

# Set number of iterations
function clp_set_number_iterations(clp_model::ClpModel, iters::Integer)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setNumberIterations Void (Ptr{Void}, Int32) clp_model.p iters
    return
end

# Get maximum number of iterations
function clp_maximum_iterations(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall maximumIterations Int32 (Ptr{Void},) clp_model.p
end

# Set maximum number of iterations
function clp_set_maximum_iterations(clp_model::ClpModel, max_iters::Integer)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setMaximumIterations Void (Ptr{Void}, Int32) clp_model.p max_iters
    return
end

# Get maximum time in seconds (from when set is called)
function clp_maximum_seconds(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall maximumSeconds Float64 (Ptr{Void},) clp_model.p
end

# Set maximum time in seconds (from when set is called)
function clp_set_maximum_seconds(clp_model::ClpModel, max_secs::Real)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setMaximumSeconds Void (Ptr{Void}, Float64) clp_model.p max_secs
    return
end

# Query whether maximum iterations or time were hit
function clp_hit_maximum_iterations(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall hitMaximumIterations Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Get the status of the problem:
#   0 - optimal
#   1 - primal infeasible
#   2 - dual infeasible
#   3 - stopped on iterations etc
#   4 - stopped due to errors
function clp_status(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall status Int32 (Ptr{Void},) clp_model.p
end

# Set the status of the problem.
function clp_set_problem_status(clp_model::ClpModel, status::Integer)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setProblemStatus Void (Ptr{Void}, Int32) clp_model.p status
    return
end

# Get the secondary status of problem - may get extended:
#   0 - none
#   1 - primal infeasible because dual limit reached
#   2 - scaled problem optimal - unscaled has primal infeasibilities
#   3 - scaled problem optimal - unscaled has dual infeasibilities
#   4 - scaled problem optimal - unscaled has both dual and primal infeasibilities
function clp_secondary_status(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall secondaryStatus Int32 (Ptr{Void},) clp_model.p
end

# Set the secondary status of the problem.
function clp_set_secondary_status(clp_model::ClpModel, status::Integer)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setSecondaryStatus Void (Ptr{Void}, Int32) clp_model.p status
    return
end

# Get the direction of optimization:
#   1 - minimize
#   -1 - maximize
#   0 - ignore
# XXX: for some reason this returns a floating point (???)
function clp_optimization_direction(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall optimizationDirection Float64 (Ptr{Void},) clp_model.p
end

# Set the direction of optimization.
# XXX: for some reason this takes a floating point argument (???)
function clp_set_optimization_direction(clp_model::ClpModel, value::Real)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setOptimizationDirection Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Get primal row solution.
function clp_get_row_activity(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    row_activity_p = @clp_ccall getRowActivity Ptr{Float64} (Ptr{Void},) clp_model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
    row_activity = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_activity), row_activity_p, num_rows * sizeof(Float64))
    return row_activity
end
clp_primal_row_solution = clp_get_row_activity

# Get primal column solution.
function clp_get_col_solution(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    col_solution_p = @clp_ccall getColSolution Ptr{Float64} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    col_solution = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(col_solution), col_solution_p, num_cols * sizeof(Float64))
    return col_solution
end
clp_primal_column_solution = clp_get_col_solution

# Get dual row solution.
function clp_get_row_price(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    row_price_p = @clp_ccall getRowPrice Ptr{Float64} (Ptr{Void},) clp_model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
    row_price = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_price), row_price_p, num_rows * sizeof(Float64))
    return row_price
end
clp_dual_column_solution = clp_get_row_price

# Get dual column solution (i.e. the reduced costs).
function clp_get_reduced_cost(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    reduced_cost_p = @clp_ccall getReducedCost Ptr{Float64} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    reduced_cost = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(reduced_cost), reduced_cost_p, num_cols * sizeof(Float64))
    return reduced_cost
end
clp_dual_column_solution = clp_get_reduced_cost

# Get row lower bounds.
function clp_get_row_lower(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    row_lower_p = @clp_ccall getRowLower Ptr{Float64} (Ptr{Void},) clp_model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
    row_lower = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_lower), row_lower_p, num_rows * sizeof(Float64))
    return row_lower
end
clp_row_lower = clp_get_row_lower

# Get row upper bounds.
function clp_get_row_upper(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    row_upper_p = @clp_ccall getRowUpper Ptr{Float64} (Ptr{Void},) clp_model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
    row_upper = Array(Float64, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(row_upper), row_upper_p, num_rows * sizeof(Float64))
    return row_upper
end
clp_row_upper = clp_get_row_upper

# Get objective coefficients.
function clp_get_obj_coefficients(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    obj_coefficients_p = @clp_ccall getObjCoefficients Ptr{Float64} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    obj_coefficients = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(obj_coefficients), obj_coefficients_p, num_cols * sizeof(Float64))
    return obj_coefficients
end
clp_objective = clp_get_obj_coefficients

# Get column lower bounds.
function clp_get_col_lower(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    col_lower_p = @clp_ccall getColLower Ptr{Float64} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    col_lower = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(col_lower), col_lower_p, num_cols * sizeof(Float64))
    return col_lower
end
clp_column_lower = clp_get_col_lower

# Get column upper bounds.
function clp_get_col_upper(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    col_upper_p = @clp_ccall getColUpper Ptr{Float64} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    col_upper = Array(Float64, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Float64}, Ptr{Float64}, Uint), pointer(col_upper), col_upper_p, num_cols * sizeof(Float64))
    return col_upper
end
clp_column_upper = clp_get_col_upper

# Get the number of elements in matrix.
function clp_getNumElements(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall getNumElements Int32 (Ptr{Void},) clp_model.p
end

# Get column starts in matrix.
# XXX TODO check the size is actually num_cols
function clp_get_vector_starts(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    vec_starts_p = @clp_ccall getVectorStarts Ptr{CoinBigIndex} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    vec_starts = Array(CoinBigIndex, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{CoinBigIndex}, Ptr{CoinBigIndex}, Uint), pointer(vec_starts), vec_starts_p, num_cols * sizeof(CoinBigIndex))
    return vec_starts
end

# Get row indices in matrix.
function clp_get_indices(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    row_indices_p = @clp_ccall getIndices Ptr{Int32} (Ptr{Void},) clp_model.p
    num_rows = @clp_ccall getNumRows Int32 (Ptr{Void},) clp_model.p
    row_indices = Array(Int32, num_rows)
    ccall(:memcpy, Ptr{Void}, (Ptr{Int32}, Ptr{Int32}, Uint), pointer(row_indices), row_indices_p, num_rows * sizeof(Int32))
    return row_indices
end

# Get column vector lengths in matrix.
function clp_getVectorLengths(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    vec_len_p = @clp_ccall getVectorLengths Ptr{Int32} (Ptr{Void},) clp_model.p
    num_cols = @clp_ccall getNumCols Int32 (Ptr{Void},) clp_model.p
    vec_len = Array(Int32, num_cols)
    ccall(:memcpy, Ptr{Void}, (Ptr{Int32}, Ptr{Int32}, Uint), pointer(vec_len), vec_len_p, num_cols * sizeof(Int32))
    return vec_len
end

# Get element values in matrix.
function clp_get_elements(clp_model::ClpModel)
    # TODO
    error("TODO")
    return # Array of Float64 values
end

# Get objective value.
function clp_get_obj_value(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall getObjValue Float64 (Ptr{Void},) clp_model.p
end
clp_objective_value = clp_get_obj_value

# Get integer information.
function clp_integer_information(clp_model::ClpModel)
    # TODO
    error("TODO")
    return # vector of chars (?)
end

# Get an infeasibility ray (nothing returned if none/wrong).
function clp_infeasibility_ray(clp_model::ClpModel)
    # TODO
    error("TODO")
    # XXX free the array returned by the ccall!
    return # array of Float64 or nothing
end

# Get an unbounded ray (nothing returned if none/wrong).
function clp_unbounded_ray(clp_model::ClpModel)
    # TODO
    error("TODO")
    # XXX free the array returned by the ccall!
    return # array of Float64 or nothing
end

# Query whether the status array exists (partly for OsiClp).
function clp_statusExists(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall statusExists Int32 (Ptr{Void},) clp_model.p
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
function clp_status_array(clp_model::ClpModel)
    # TODO
    error("TODO")
    return # pointer to unsigned chars (?)
end

# Copy in the status vector
# XXX copyin rather than copy_in, WTF?
# XXX input vector of unsigned chars (?)
function clp_copyin_status(clp_model::ClpModel, status_array::Vector{Uint8})
    # TODO
    error("TODO")
    return
end

# Get variable basis info.
function clp_get_column_status(clp_model::ClpModel, sequence::Int32)
    # TODO
    error("TODO")
    return # integer
end

# Get row basis info.
function clp_get_row_status(clp_model::ClpModel, sequence::Int32)
    # TODO
    error("TODO")
    return # integer
end

# Set variable basis info (and value if at bound).
function clp_set_column_status(clp_model::ClpModel, sequence::Int32, value::Int32)
    # TODO
    error("TODO")
    return
end

# Set row basis info (and value if at bound).
function clp_set_row_status(clp_model::ClpModel, sequence::Int32, value::Int32)
    # TODO
    error("TODO")
    return
end

# Set user pointer (used for generic purposes)
function clp_get_user_pointer(clp_model::ClpModel)
    # TODO
    error("TODO")
    return # Ptr{Void}
end

# Get user pointer (used for generic purposes)
function clp_set_user_pointer(clp_model::ClpModel, pointer::Ptr{Void})
    # TODO
    error("TODO")
    return
end

# Pass in callback function.
# Message numbers up to 1000000 are Clp, Coin ones have 1000000 added
# TODO
# COINLIBAPI void COINLINKAGE Clp_registerCallBack(Clp_Simplex * model,
#        clp_callback userCallBack);

# Unset callback function.
function clp_clear_call_back(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall clearCallBack Void (Ptr{Void},) clp_model.p
    return
end

# Get amount of print out:
#   0 - none
#   1 - just final
#   2 - just factorizations
#   3 - as 2 plus a bit more
#   4 - verbose
#   above that: 8,16,32 etc just for selective debug.
function clp_log_level(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall logLevel Int32 (Ptr{Void},) clp_model.p
end

# Set amount of print out.
function clp_set_log_level(clp_model::ClpModel, value::Integer)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setLogLevel Void (Ptr{Void}, Int32) clp_model.p value
    return
end

# Get length of names (0 means no names).
function clp_length_names(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    return @clp_ccall lengthNames Int32 (Ptr{Void},) clp_model.p
end

# Return an array with a row name.
# XXX deviation from Clp API syntax, which fills a user-provided
# array of chars:
#   void Clp_rowName(Clp_Simplex * model, int iRow, char * name);
function clp_row_name(clp_model::ClpModel, row::Integer)
    _jl_clp__check_clp_model(clp_model)
    _jl_clp__check_row_is_valid(clp_model, row)
    size = @clp_ccall lengthNames Int32 (Ptr{Void},) clp_model.p
    row_name_p = pointer(Array(Uint8, size))
    @clp_ccall rowName Void (Ptr{Void}, Int32, Ptr{Uint8}) clp_model.p (row-1) row_name_p
    row_name = string(row_name_p)
    return row_name
end

# Return an array with a column name.
# XXX deviation from Clp API syntax, which fills a user-provided
# array of chars:
#   void Clp_columnName(Clp_Simplex * model, int iColumn, char * name);
function clp_column_name(clp_model::ClpModel, col::Integer)
    _jl_clp__check_clp_model(clp_model)
    _jl_clp__check_col_is_valid(clp_model, col)
    size = @clp_ccall lengthNames Int32 (Ptr{Void},) clp_model.p
    col_name_p = pointer(Array(Uint8, size))
    @clp_ccall columnName Void (Ptr{Void}, Int32, Ptr{Uint8}) clp_model.p (col-1) col_name_p
    col_name = string(col_name_p)
    return col_name
end

# General solve algorithm which can do presolve.
function clp_initial_solve(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall initialSolve Int32 (Ptr{Void},) clp_model.p
end

# Dual initial solve.
function clp_initial_dual_solve(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall initialDualSolve Int32 (Ptr{Void},) clp_model.p
end

# Primal initial solve.
function clp_initial_primal_solve(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall initialPrimalSolve Int32 (Ptr{Void},) clp_model.p
end

# Barrier initial solve.
function clp_initial_barrier_solve(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall initialBarrierSolve Int32 (Ptr{Void},) clp_model.p
end

# Barrier initial solve, no crossover.
function clp_initial_barrier_no_cross_solve(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall initialBarrierNoCrossSolve Int32 (Ptr{Void},) clp_model.p
end

# Dual algorithm.
function clp_dual(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall dual Int32 (Ptr{Void},) clp_model.p
end

# Primal algorithm.
function clp_primal(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall primal Int32 (Ptr{Void},) clp_model.p
end

# Solve the problem with the idiot code.
function clp_idiot(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall idiot Int32 (Ptr{Void},) clp_model.p
end

# Set or unset scaling:
#  0 - off
#  1 - equilibrium
#  2 - geometric
#  3 - auto
#  4 - dynamic (later)
function clp_scaling(clp_model::ClpModel, mode::Integer)
    _jl_clp__check_clp_model(clp_model)
    if !(0 <= mode <= 4)
        error("Invalid clp scaling mode $mode (must be between 0 and 4)")
    end
    @clp_ccall scaling Void (Ptr{Void}, Int32) clp_model.p mode
    return
end

# Get scaling flag.
function clp_scaling_flag(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    return @clp_ccall scalingFlag Int32 (Ptr{Void},) clp_model.p
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
function clp_crash(clp_model::ClpModel, gap::Float64, pivot::Int32)
    _jl_clp__check_clp_model(clp_model)
    if !(0 <= pivot <= 2)
        error("Invalid clp crash pivot value $pivot (must be between 0 and 2)")
    end
    return @clp_ccall crash Int32 (Ptr{Void}, Float64, Int32) clp_model.p gap pivot
end

# Query whether the problem is primal feasible.
function clp_primal_feasible(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall primalFeasible Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether the problem is dual feasible.
function clp_dual_feasible(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall dualFeasible Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Get dual bound.
function clp_dual_bound(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall dualBound Float64 (Ptr{Void},) clp_model.p
end

# Set dual bound.
function clp_set_dual_bound(clp_model::ClpModel, value::Real);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setDualBound Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Get infeasibility cost.
function clp_infeasibility_cost(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall infeasibilityCost Float64 (Ptr{Void},) clp_model.p
end

# Set infeasibility cost.
function clp_set_infeasibility_cost(clp_model::ClpModel, value::Real);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall setInfeasibilityCost Void (Ptr{Void}, Float64) clp_model.p value
    return
end

# Get Perturbation. Values are:
#   50  - switch on perturbation
#   100 - auto perturb if takes too long (1.0e-6 largest nonzero)
#   101 - we are perturbed
#   102 - don't try perturbing again
# The default is 100; others are for playing.
function Clp_perturbation(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall perturbation Int32 (Ptr{Void},) clp_model.p
end

# Set Perturbation.
function clp_set_perturbation(clp_model::ClpModel, value::Integer);
    _jl_clp__check_clp_model(clp_model)
    if !(value == 50 || value == 100 || value == 101 || value == 102)
        error("Invalid clp perturbation value: $value (must be one of 50,100,101,102)")
    end
    @clp_ccall setPerturbation Void (Ptr{Void}, Int32) clp_model.p value
    return
end

# Get current (or last) algorithm.
function clp_algorithm(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall algorithm Int32 (Ptr{Void},) clp_model.p
end

# Set algorithm.
function clp_set_algorithm(clp_model::ClpModel, value::Integer);
    _jl_clp__check_clp_model(clp_model)
    # XXX which values of the algorithm are valid ???
    @clp_ccall setAlgorithm Void (Ptr{Void}, Int32) clp_model.p value
    return
end

# Get the sum of dual infeasibilities.
function clp_sum_dual_infeasibilities(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall sumDualInfeasibilities Float64 (Ptr{Void},) clp_model.p
end

# Get the number of dual infeasibilities.
function clp_number_dual_infeasibilities(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall numberDualInfeasibilities Int32 (Ptr{Void},) clp_model.p
end

# Get the sum of primal infeasibilities.
function clp_sum_primal_infeasibilities(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall sumPrimalInfeasibilities Float64 (Ptr{Void},) clp_model.p
end

# Get the number of primal infeasibilities.
function clp_number_primal_infeasibilities(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall numberPrimalInfeasibilities Int32 (Ptr{Void},) clp_model.p
end

# Save model to file, returns 0 if success.  This is designed for
# use outside algorithms so does not save iterating arrays etc.
# It does not save any messaging information.
# Does not save scaling values.
# It does not know about all types of virtual functions.
function clp_save_model(clp_model::ClpModel, file_name::Vector{Uint8});
    # todo
    error("todo")
    return # Int32 [Bool]
end

# Restore model from file, returns 0 if success,
# deletes current model.
function clp_restore_model(clp_model::ClpModel, file_name::Vector{Uint8});
    # todo
    error("todo")
    return # Int32 [Bool]
end

# Just check solution (for external use) - sets sum of
# infeasibilities etc.
function clp_check_solution(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall checkSolution Void (Ptr{Void},) clp_model.p
    return
end

# Query whether there are numerical difficulties.
function clp_is_abandoned(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isAbandoned Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether optimality is proven.
function clp_is_proven_optimal(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isProvenOptimal Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether primal infeasiblity is proven.
function clp_is_proven_primal_infeasible(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isProvenPrimalInfeasible Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether dual infeasiblity is proven.
function clp_is_proven_dual_infeasible(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isProvenDualInfeasible Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether the given primal objective limit is reached.
function clp_is_primal_objective_limit_reached(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isPrimalObjectiveLimitReached Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether the given dual objective limit is reached.
function clp_is_dual_objective_limit_reached(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isDualObjectiveLimitReached Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Query whether the iteration limit is reached
function clp_is_iteration_limit_reached(clp_model::ClpModel)
    _jl_clp__check_clp_model(clp_model)
    ret = @clp_ccall isIterationLimitReached Int32 (Ptr{Void},) clp_model.p
    return ret != 0
end

# Get the direction of optimization:
#   1  - minimize
#   -1 - maximize
#   0  - ignore
# XXX the return value is floating point, WTF???
function clp_get_obj_sense(clp_model::ClpModel);
    _jl_clp__check_clp_model(clp_model)
    @clp_ccall getObjSense Float64 (Ptr{Void},) clp_model.p
end

# Set the direction of optimization.
# XXX the value is floating point, WTF???
function clp_set_obj_sense(clp_model::ClpModel, objsen::Real);
    _jl_clp__check_clp_model(clp_model)
    if !(objset == -1 || objset == 0 || objsen == 1)
        error("Invalid clp objsense $objsense (should be -1,0 or 1)")
    end
    @clp_ccall setObjSense Void (Ptr{Void}, Float64) clp_model.p objsen
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
