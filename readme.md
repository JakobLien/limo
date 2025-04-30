# Limo masters thesis
This will just be short comments I make along the way, to myself. Try reading the thesis for more help and discussion. 

- Killing all python processes that hang: `pkill -9 python`
- To connect to limo using VS Code, use an earlier version of VS Code and of the extention. Disable automatic updates. See [here](https://stackoverflow.com/questions/69626028/the-remote-host-may-not-meet-vs-code-servers-prerequisites-for-glibc-and-libstd)
- To run python as a priority process, in the context of OS scheduling, bruk `cd ../../Desktop/limo; sudo -v && ( sleep 0.2 && sudo renice -n -20 -p $(pgrep -n python3) ) & python3 main.py;`. This reduces the number of jumps in performance:)
  - This command is more complex than I would like because we have to give the sudo password to renice which runs in the background. 
  - And we were unable to just use nice because then the pygame window wouldn't show for some reason. 
  - And we have to keep main.py at the end to be able to stop the command by using CTRL+C
