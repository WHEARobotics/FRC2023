# Preparing for a development session

You need to have a computer set up as in DEVELOPERS_START_HERE.md.  If not, go to that file first.

Open an Anaconda prompt (Start menu, start typing "anaconda" in the search box).

In the Anaconda prompt, change directory (folder) to the folder with your code.  
This is has been set up on the desktop on our computers. 
```cd Desktop\frc2023``` (relative to the user's folder)
or
```cd c:\Users\<username>\Desktop\frc2023``` (specifying the full folder name, but you do need the correct username)


Activate the python environment we set up.  This tells this command window to use the Python version
and libraries that we had installed in the start here document.
```conda activate frc```

In most cases you want to make sure you are using the latest code, which might not have been written using this 
computer.  Hopefully, it was pushed to the GitHub cloud repository.  To get it (pull it from the repository), 
type the following in the command window:

```git pull``

# Editing code
Use VSCode to edit your code.


# Deploying code to the robot

Check the robot:
* If you have changed wiring, are the wire polarities correct?
* Are all the wires connected?
* Make sure there is nothing that could get in the way if motors move.  (wires, wrenches, fingers)

1. Power the robot by turning on the breaker.
2. Open driver station software on PLTW1.
3. Connect PLTW1 to the Wi-Fi to the access point on the robot (access point name 3881).
4. Check your code again.  Are there any spelling errors?
5. If you are using a different computer from PLTW1 to write code, also connect that computer's Wi-Fi to the access point on the robot (access point name 3881).

TODO 2024-06-07: I think this is wrong now. I think you need to use `robotpy` command. 
Deploy your code:
```python robot.py deploy```

Look for the message that it deployed successfully.

# When you are done developing

When you have the code in a reasonable stopping point, you want to make a "commit", or take a snapshot of the state of the local repository,
and push it to the cloud.  Then you or the next person can pull the latest changes.  

It is a 3-step process:

1. ```git add robot.py```, does what is called "staging" the changed file robot.py for a commit.  If you have changed other files that you want
to commit, also add them the same way.

2. ```git commit -m "Descriptive commit message goes here."```, this makes the commit using the file(s) you staged.  The message should describe what you did, or the status of the code.

3. ```git push origin```, this pushes the latest commit to "origin", which is our GitHub repository.


# Troubleshooting

If you deploy successfully, but you see that the "Robot Code" indicator on the Driver Station software goes back and forth between red and green, you may have
a run-time error in your code. 

1. Open the FRC Driver Station window
1. Click the gear icon on the left-hand side
1. Click the gear icon that appears on the right-hand side

![](docs/console_view_1.png)

4. In the drop-down menu that appears, choose "View console"

![](docs/console_view_2.png)

You should see information printed from the robot code here. You should see here some indication of the error. Look for Python error dump traces.  There may be several lines showing errors in library code we are calling, but one of them will
indicate a line number in robot.py.  That's the one to look at.

The below image shows the console view after the robot code was deployed successfully. You can
see information coming from **nt* (the network tables library) and from **robot** (our code).
In this case, the robot code is properly seeing the limelight data in the `limelight` networktabe.

![](docs/console_view_3.png)

