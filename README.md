# VehicleSim
VehicleSim

<img src="https://github.com/forrestlaine/VehicleSim/blob/main/parked_cars.png" />

# Loading / instantiating code

It is recommended to start julia with multiple threads, since many concurrent tasks will be executing. 

```
julia --project --threads=auto
```

```julia
(VehicleSim) pkg> instantiate
(VehicleSim) pkg> add https://github.com/forrestlaine/MeshCat.jl
(VehicleSim) pkg> add https://github.com/forrestlaine/RigidBodyDynamics.jl
```

```julia
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
julia> using Sockets # to allow ip strings
julia> keyboard_client(ip"1.2.3.4") # ip address specified by @info statement when starting server
[ Info: Client accepted.
[ Info: Client follow-cam can be connected to at 1.2.3.4:8713
[ Info: Press 'q' at any time to terminate vehicle.
```

# Shutting down server
```julia
julia> shutdown!(s)
```

# Writing an autonomous vehicle client

The file example_project.jl outlines a recommended architecture for ingesting sensor messages and creating vehicle commands.
