module VehicleSim

using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using Infiltrator
using LinearAlgebra
using SparseArrays
using StaticArrays
using Ipopt
using PATHSolver
using Symbolics

include("view_car.jl")
include("objects.jl")
include("sim.jl")

export view_car, sim_car, simulate, test_simulate

end # module VehicleSim
