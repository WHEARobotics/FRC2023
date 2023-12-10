# Machine Learning Developer Configuration

It's always a challenge setting up the tools and libraries you need to just get started. For FRC, we've used Python's `conda` tool to create an environment, as detailed in [Developer's Start Here](./DEVELOPERS_START_HERE.md).

For ML, we're going to try a different approach. [Docker](https://www.docker.com/) is a system that builds a snapshot of a development system (a Docker **image**). This image, when run, creates an isolated system, very much like a local cloud machine, called a Docker **container**. An image is like an installation program, a container is like your running operating system (complete with local files that persist when you shut it off and come back when you restart it).

The promise is that you:

1) Download or build a Docker image
2) Run the image to start your local Docker container
3) You can immediately start developing with the container

Is this easier than setting up a `conda` environment? Theoretically! But let's see. 

## Install Docker Desktop

Go to [https://docs.docker.com/get-docker/](https://docs.docker.com/get-docker/) and download Docker Desktop for Windows. (If you want to use a Mac for development, we'll have to customize the Dockerfile, which is no big deal.)

Once you install it and run it, you'll see something like this, with tabs for Containers, Images, etc. 

![Docker Desktop](media/ml_start/docker_desktop.png)

## Search for the "Hello World" Docker image

Enter "Search" in the search bar and look for the result called `hello-world` that has over a billion downloads and more than 2K stars. 

![h-w image](media/ml_start/hello_world.png)

## Pull (download) the image

Hover your mouse over the image and you should see: 

![pull](media/ml_start/pull.png)

Press the "Pull" button to download the image. This will be fast with this small image but generally images are **several gigabytes** in size. Our image, for instance, is about 8GB. 

## Run the image, creating a container

Now press the "Run" button. You should see something like:

![hello output](media/ml_start/hello_run.png)

1) Notice that you're in the **Containers** tab. 
2) Your container will have a different name than mine (`eager_euclid`) but the image from which it was created will be the same: `hello-world:latest`. In addition to the weird, but easily remembered, name is a complex ID for this container (yours will be different than my `2b379c903fd9`)
3) The **Status** is "Exited" because this computer runs, outputs the messages you see in the **Logs**, and then shuts down. 
4) By pressing these buttons, you can stop a running container, start a stopped container, restart a running container, or delete the container. 

Go ahead and delete this container by pressing garbage can icon. 

By deleting the **container** you're getting rid of "this particular computer". But since you have the **image**, you can "make another one" easily. 

## Examine the image and create a new container

Click the **Images** tab in the left-hand panel. You'll see something like this: 

![images tab](media/ml_start/image_tab.png)

1) You should be in the **Images** tab 
2) You will see a list of images you've downloaded, which should only be the `hello-world` container. 
3) If you deleted the container, the **Status** of this container will be "Unused". If there were one or more containers that relied on the image, the status would be "In use," like it is for my `wildme/nginx` image. 
4) You can take some actions on an image. You can only delete an image that has no containers relying on it (that is, it's status is "Unused"). Instead, press the **Run** icon to create a new **container**. 

You'll see something _similar_ to what you saw when you ran the image the first time, but notice that the name and identity of this container is different. If you click **Run** on an _image_ you get a **new** container. You can have many containers running on the same image. That's good if you're running a data center, it's probably not what you want at WHEA! 

### Yes: Run the same container over and over with the **Run** icon on the **Containers** tab
### No: Create new containers over and over with the **Run** icon on the **Images** tab  

## Pull the WHEA Machine Learning and Robotics image

When you can be sure your machine will have at least an hour to work, open a Terminal and switch to this repository's `Docker/` directory. Run:

`docker pull lobrien/whea:frc_cpu`

This retrieves the (big) image I set up from Docker Hub. 

**This will take a long time**

## Run the whea machine-learning image

When the image is downloaded, return to the terminal. Switch to the `src/` directory in this repository and run the image. Replace `{complete path}` below with your complete path to the `src/` directory:

`docker run -it -p 8888:8888 -v {complete path}:/app/src frc_cpu`

For instance: 

`docker run -it -p 8888:8888 -v c:\Users\lobri\Documents\src\FRC2023\src:/app/src frc_cpu`

## Did all of that work? 

Open a Web browser and go to [http://localhost:8888/](http://localhost:8888). You should see something like:

![jupyter](media/ml_start/jupyter.png)

In the left-hand tab, open the `machine-learning/` directory. 

**RIGHT**-click on the `01-xor.py` file. In the popup-menu, choose "Open as..."|"Jupytext notebook"

![jupytext](media/ml_start/jupytext.png)

Select "Python 3 (ipykernel)" and add a check to the "Always start the preferred kernel" box. 

![xor](media/ml_start/run_all.png)

From the "Run" menu, choose "Run all cells..."

If things start going, pat yourself on the back!!!! 

## Review

To complete this *long* process, you:

* Installed **Docker** Desktop
* Downloaded and ran a "Hello-World" **image**, creating one or more **containers** along the way
* Pulled the WHEA docker image from Docker Hub
* Ran the Docker image, creating a **container** in which is installed PyTorch, Jupyter, and other things used in ML development
* Opened a Python file as a Jupytext notebook, which allows you to interactively write, run, and graph Python

### Sidebar: Machine Learning really benefits from a machine with an NVidia GPU

The company NVidia has a very big lead in the low-level software that does machine learning operations. If you have an NVidia GPU on your Windows machine, you can take advantage of their Compute Unified Device Architecture (CUDA) libraries. These libraries do many ML operations anywhere from several to an order-of-magnitude faster than can be done otherwise. 

Even if your laptop has a built-in GPU, it probably *doesn't* have a GPU from NVidia. If you have a Windows machine at home, especially if it can run games pretty well, it probably *does* have an NVidia GPU. You might want to install Docker on your home machine, run the first couple ML programs, and see if it you get a huge speedup.

