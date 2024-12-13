# Devs Rage Over Nonsense Errs (D.R.O.N.E) drone.

Simply insert
```
[dependencies]
d-r-o-n-e_drone = { git = "https://github.com/AP-2024-25-D-R-O-N-E/drone.git" }
```
in your cargo.toml file.


For any troubleshooting or issues feel free to join the telegram group chat: https://t.me/+bIpJW7DWk9gyNWY0


## Enabling logging
1. Add the `simple_logger` crate to `cargo.toml`. 
```toml
simple_logger = "5.0.0"
```
2. Add the following lines at the start of your program
```rust
 SimpleLogger::new()
     .without_timestamps()
     .env()      
     .init()
     .unwrap();
```
Note that the code above should run only once.

3. To modify the granularity of the logging, you can set the `RUST_LOG` environment variable. You can either: 
	* Set it "locally" before executing your project: 
	 ```bash
	 RUST_LOG="<level>" cargo run
	 ```
	 * Set it locally to bash session: 
	  ```bash
	  export RUST_LOG="<level>"
	  ...
	  cargo run
	  ```
	  * Set it inside the `<project_root>/.cargo/config.toml` file (create it if not present): 
	  ```toml
	[env]
	RUST_LOG = "Debug"
	  ```
The `RUST_LOG` variable can have the following values, in increasing order of granularity: `Error - Warn - Info - Debug - Trace`. 

