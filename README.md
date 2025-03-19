# VehicleSim
VehicleSim

<img src="https://github.com/forrestlaine/VehicleSim/blob/main/parked_cars.png" />

# Loading / instantiating code

It is recommended to start julia with multiple threads, since many concurrent tasks will be executing. Important: you must use julia 1.9.3 for all functionality to work properly. 

```
julia +1.9.3 --project --threads=auto
```

```julia
(VehicleSim) pkg> instantiate
(VehicleSim) pkg> add https://github.com/forrestlaine/MeshCat.jl
(VehicleSim) pkg> add https://github.com/forrestlaine/RigidBodyDynamics.jl
```

You may need to restart Julia at this point before proceeding.

```julia
julia> using VehicleSim
```

# Running Simulation

```julia
julia> server();
┌ Info: MeshCat server started. You can open the visualizer by visiting the following URL in your browser:
└ http://1.2.3.4:8700
┌ Info: ********************
│       CONNECTING TO SERVER
│       ********************
│         -Connect a keyboard client by running (in a new REPL):
│             using Vehicle Sim, Sockets; keyboard_client(ip"1.2.3.4")
└         -Port for manual clients is 4444
[ Info: Target for vehicle 1: 40
┌ Info: ***************
│       VIEWER COMMANDS
│       ***************
│             -Make sure focus is on this terminal window. Then:
│             -Press 'q' to shutdown server. 
│             -Press '0' to switch to bird's-eye view and release controls to user.
│             -Press a number '1'-'9' to view the follow-cam for the associated vehicle. Will default to '0' if vehicle doesn't exist.
└             -Use the 'shift' modifier to follow-cam from top-down (e.g. '!' for vehicle 1).
[ Info: Waiting for client
```

This will spin up the server / simulation engine. For now, the server will instantiate a single vehicle. 

# Connecting a keyboard client

In a separate REPL, you can connect to the server with a keyboard client, allowing you to manually drive a vehicle.

```julia
julia> using VehicleSim, Sockets # to allow ip strings
julia> keyboard_client(ip"1.2.3.4") # ip address specified by @info statement when starting server
[ Info: Client accepted.
[ Info: Client follow-cam can be connected to at 1.2.3.4:8713
[ Info: Press 'q' at any time to terminate vehicle.
```
