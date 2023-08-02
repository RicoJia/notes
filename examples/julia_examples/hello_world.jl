# # This hello world example is from this video: 
# #  https://www.youtube.com/watch?v=8h8rQyEpiZA&list=PLP8iPy9hna6SCcFv3FvY_qjAmtTsNYHQE&index=9

# ##############################################
# # Basics
# ##############################################

# # Pkg manager
# # Install Ijulia for running it in Jupyter Notebook:
# # julia -> using Pkg -> Pkg.add("IJulia")
# # Or, "]" for entering the pkg repl -> add IJulia
# # Pkg.status() or status in pkg repl shows what packages are installed
# # Pkg.update(PKG) or Pkg.rm("pkg")

# # Jupyter Notebook:
# # ?command to see intro; ;ls to run bash command ls

# # Variables
# my_var = 12
# # see int64
# println(typeof(my_var), ", exponential: ", 2^3)
# println("modulus: ", 10%3)

# # strings 
# s1 = "quotes"
# s2 = """ tripple quotes allows "this" """
# s3 = "dummy string"
# # Concatenation
# println(s1*s2*s3)
# println("string interpolation is to use \$ : $my_var")

# '' only used on chars
char1 = 'c'

# If you don't want printouts, append ; at the end of the function
# Random.seed!(0);

# in julia, no time.sleep
# sleep(1)

# # tuple is the same in python. But Julia index starts from 1
# tup = (1,2,3)
# println(tup[1])
# # Named tuples:
# named_tup = (bird=1, dog=2)
# println("named tuple: ", named_tup[1], named_tup.bird)

# # Dictionaries:
# di = Dict("bird"=>1, "dog"=>2)
# di["new_key"] = 3
# println(di)
# pop!(di, "new_key")
# println("after pop!", di)

# # array
# # it's int64
# arr1 = [1,2,3]
# # here it's any
# arr2 = [1,2, "3"]
# push!(arr1, 12)
# println("after push!", arr1)
# pop!(arr2)
# println("after pop!", arr1)

# # while loop
# some_n=10
# while some_n<0
#     # Julia requires global scope
#     global some_n
#     some_n+=1
#     println(some_n)
# end
# ##############################################
# # Math
# ##############################################
# 1. Max and argmax
arr = [1,2,3,4,5]
println("max element: ",maximum(arr)," . max index: ",findmax(arr))
# 2. Broadcasting
arr1 = [5,6,7,8,9]
println("If all elements are within tolerance of 2: ", all(abs.(arr .- arr1) .< 2))

# Reference
arr = arr1
arr1[1] = 100
println("arr also changed, because we have a reference: ", arr[1])

# 
for element in arr
    println(element)
end

# # Or a julia syntactic sugar
# for r in 1:3, w in 1:4
#     A[r,w]=100
# end
# println(A)

# # Array comprehension
# # Here we have a 2D array, by default, it's row, column in A
# A = [i+j for i in 1:3, j in 4:5]
# println("array comprehension: ", A)

# # copy array needs copy(), else, it's simply a shallow copy.
# # to recursively copy all elements in a dictionary, do deepcopy()
# cp_arr1 = copy(arr1)
# deepcopy(arr2)

# while length(cp_arr1) <= 1
#     push!(cp_arr1,2)
# end

# # can specify step size this way
# for i in 1:3:10
#     println(i)
# end

# # Can create an Array
# A = fill(0, (3,4))
# # Note: these are inclusive, [1:3]
# for r in 1:3
#     for w in 1:4
#         #note it's A[r,w]
#         A[r, w] = 12
#     end
# end
# println(A)


# # if elseif else
# N = 15
# if (N%3==0) && (N%5==0)
#     println("fizzbuzz")
# elseif N%3==0
#     println("fizz")
# elseif N%5==0
#     println("buzz")
# end

# # Ternary: note: space required
# 1>2 ? println(">") : println("<")

# # & vs &&
# # Note it's true, not True
# # no short circuit
# false & (println("& has no short circuit here"); true)
# # there is short circuit
# false && (println("hi"); true)
# true | (println("| has no short circuit here"); true)
# true || (println("| has short circuit here"); true)

# ##############################################
# # Function
# ##############################################

# # will automatically return the last line result
# function foo(i)
#     i=1
#     # or return i
# end
# println("print last var", foo(2))
# # lambda function
# f3 = x -> x+1
# println("lambda function", f3(3))
# # shorthand Function
# f(x, y) = 3x + 4y
# A = [1, 2, 3]
# B = [10, 20, 30]
# C = f(A, B)
# println(C)  # Outputs: [43, 86, 129]

# ##############################################
# # Matrix
# ##############################################
# # matrix multiplication?
# mat1 = fill(1, (3,3))
# println("matrix multiplying by itself: ", mat1^2)

# #mutating function: by convention, function name should end in !
# function update_input!(input)
#     # update input here
# end


# # Broadcasting
# println("without broadcasting: ", [1,2,3] + [4,5,6])
# # But we can't do , [1,2,3] + [4,5,6]'
# # With broadcasting, it's like 3 rows of [1,2,3], added with an element in the column vector
# println("with broadcasting: ", [1,2,3] .+ [4,5,6]')

# # If a plot with multiple lines doesn't show properly, then it's probably because 
# # the line data size do not match up

##############################################
# Benchmarking Sum, Calling Raw C, python built in
##############################################
using BenchmarkTools
double_array = [1.0,2.0,3.1,4.2,5.3]
# @ is a macro
@time sum(double_array)
benchmark = @benchmark sum(double_array)
println("Julia built in sum benchmarking: ", benchmark)

# # We can even have a hand writen piece of code
using Libdl
C_code = """
#include <stddef.h>
// size_t is included in stddef.h
double c_sum (size_t n, double* array){
    double sum = 0.0;
    for (size_t i = 0; i < n; i++){
        sum += array[i];        
    }
    return sum;
}
"""
# pipe code to gcc, then compile it
const Clib = tempname()
open(`gcc -fPIC -O3 -msse3 -xc -shared -o $(Clib * "." * Libdl.dlext) -`, "w") do f
    print(f, C_code)
end
c_sum(X::Array{Float64}) = ccall(("c_sum", Clib), Float64, (Csize_t, Ptr{Float64}), length(X), X)
# println(c_sum(double_array))
@time c_sum(double_array)
# benchmark = @benchmark c_sum(double_array)
# println("Handwritten C sum benchmarking: ", benchmark)

# Using C-fastmath, which uses SIMD (Single Instruction, Multiple Data)
# That is, same operations on multiple data points. 
# A matrix's elements can be added simultaneously. It's an instruction set. It's a parallelism
# on an instruction level, RUN ON A SINGLE CORE. E.g., load 8 64-bit float number into a SIMD register.
# Then, the SIMD instruction can add the 8 numbers simultaneously.
const Clib_fastmath = tempname()
open(`gcc -fPIC -O3 -msse3 -xc -shared -ffast-math -o $(Clib_fastmath * "." * Libdl.dlext) -`, "w") do f
    print(f, C_code)
end
c_sum_fastmath(X::Array{Float64}) = ccall(("c_sum", Clib_fastmath), Float64, (Csize_t, Ptr{Float64}), length(X), X)
@time c_sum_fastmath(double_array)
# benchmark = @benchmark c_sum_fastmath(double_array)
# println("Handwritten C sum fastmath benchmarking: ", benchmark)

# Python built in sum
using PyCall
pysum = pybuiltin("sum")
@time pysum(double_array)
# benchmark = @benchmark pysum(double_array)
# println("python built-in benchmarking", minimum(benchmark.times))

# Use numpy
using Conda
numpy_sum = pyimport("numpy")["sum"]
@time numpy_sum(double_array)
# benchmark = @benchmark numpy_sum(double_array)
# println("python numpy benchmarking", minimum(benchmark.times))

# Handwritten python function
# MUST NOT have space in py"""
py"""
def py_sum(A):
    s = 0.0
    for a in A:
        s += a
    return s
"""
py_sum = py"py_sum"
py_sum(double_array)

# @which py_sum 

# Julia handwritten, with SIMD to "vectorize" loops
# SIMD loop must be: inner loop, single instruction can load multiple numbers at once
function add_simd(A::Array{Float64})
    s = 0.0
    @simd for a in A
        s += a
    end
end
@time add_simd(double_array)
