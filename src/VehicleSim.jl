module VehicleSim

using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using Infiltrator
using LinearAlgebra
using Rotations
using SparseArrays
using Ipopt
using PATHSolver
using Symbolics

include("view_car.jl")
include("sim.jl")

export view_car, sim_car, simulate

end # module VehicleSim
