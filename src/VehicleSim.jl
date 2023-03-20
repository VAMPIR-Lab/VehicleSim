module VehicleSim

using ColorTypes
using GeometryBasics
using MeshCat
using MeshCatMechanisms
using RigidBodyDynamics
using Infiltrator
using LinearAlgebra
using SparseArrays
using Sockets
using Serialization
using StaticArrays
using DifferentialEquations
using Ipopt
using PATHSolver
using Symbolics

include("view_car.jl")
include("objects.jl")
include("sim.jl")
include("control.jl")
include("sink.jl")
include("map.jl")

export view_car, sim_car, simulate, test_simulate, get_vis, server, CarConfig, client

end
