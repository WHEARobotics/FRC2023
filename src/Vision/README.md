# WHEA Vision

## 01-DetectAndDisplay

This project detects AprilTags and draws targets on them. 
To use this program:

1. Print the file **AprilTags_1_and_2.pdf**. Use heavy paper, 
thin cardboard, or mount the paper on cardboard. Tag detection
seems to be quite sensitive to surface flatness.
2. This program requires a computer with a Webcam
3. Switch into the **01-DetectAndDisplay** subdirectory
4. Run `python DetectAndDisplay.py`
5. The first time you run this, it may request access to the
camera and fail. Once you grant access, the program should run ok.
6. Place the apriltag target(s) so the camera can see them.

Notice that detection is far from guaranteed! Try to find situations
where detection is better or worse: 

* Does detection work better in very bright light, or is 
"good enough" light good enough? 
* If there is a lot of contrast in a scene, bright areas and
dark areas, does that affect detection? 
* How does distance affect detection? The apriltags on the 
field will be 6"x6". If you print the PDF at 100%, the apriltags
will be scaled to 20% or 40% of that size. 

**NOTE** FRC competitions are *very* dynamic, visually. There 
are lots of flashing lights, colored lights, etc. 


These apriltags will be 66" apart on the field:

| ID | X      | Y | Z |
| ---|--------|---|---|
| 1  | 610.77 | 42.19 | 18.22 |
| 2 | 610.77 | 108.19 | 18.22 | 

| Scale | Distance in Y | Scaled Z height |
| ---   | ------------- | --------------- |
| 1/5 | 13.2" | 3.64" |
| 2/4 | 26.4" | 7.28" |

