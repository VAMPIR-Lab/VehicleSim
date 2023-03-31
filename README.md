# VehicleSim
VehicleSim

# Loading / instantiating code

```julia
(VehicleSim) pkg> instantiate
(VehicleSim) pkg> add https://github.com/forrestlaine/MeshCat.jl
(VehicleSim) pkg> add https://github.com/forrestlaine/RigidBodyDynamics.jl

julia> using VehicleSim
```

# Running Simulation
```julia
julia> s = server();
[ Info: Server can be connected to at 1.2.3.4 and port 4444
[ Info: Server visualizer can be connected to at 1.2.3.4:8712
```

This will spin up the server / simulation engine. For now, the server will instantiate a single vehicle. 

# Connecting a keyboard client

```julia
julia> keyboard_client(ip"1.2.3.4")
[ Info: Client accepted.
[ Info: Client follow-cam can be connected to at 1.2.3.4:8713
[ Info: Press 'q' at any time to terminate vehicle.
```

# Shutting down server
```julia
julia> shutdown!(s)
```
