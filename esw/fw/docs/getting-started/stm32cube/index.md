# STM32Cube\*

## About

STM32CubeMX and STM32CubeCLT allow the user to write, compile, and flash code to the STM32 microcontroller!
STM32CubeCLT contains the GCC compiler and GDB debugging tool for firmware compilation and debugging, and
STM32CubeMX contains the interface for configuring the microcontroller and project environment, and provides
a very powerful interface for automatic code generation, allowing the user to initialize an entire module with
a few clicks of a button, and have that code show up automatically in the main file.

!!! note
    If you've taken EECS 373, you have used STM32CubeIDE, which is an IDE that packages both of these tools
    together. However, STM32CubeIDE is a very heavy program, so we choose to use the lighter weight STM32CubeMX
    and STM32CubeCLT tools in combination with CMake and your favorite text editor or IDE.

## Downloading and Installing CubeMX

### 1. Download CubeMX

1. Go to the CubeMX [download page](https://www.st.com/en/development-tools/stm32cubemx.html) and scroll down to "Get Software."
   ![get cubemx](get-cubemx.webp)

2. **_For this walkthrough, we will be using the Linux installer for UBUNTU USERS._**
   Click on "Select Version", then select the **newest version** for the **_correct OS you are running!!!_**

3. Click on Accept for the License Agreement pop up.

4. You will then be prompted to log in, create an account, or continue as a guest. You will need a
   MyST account in the future, so it is best to create one now. You may use any email address.

5. After logging in and being brought back to the download page, the download should start automatically.
   Scroll down and select the correct version from "Get Software" if it does not.

### 2. Install CubeMX

**Note**: This install guide is for **Linux**. Please make sure to install the correct version
for the OS you are running.

1\. Open a new terminal ([guide](https://www.howtogeek.com/686955/how-to-launch-a-terminal-window-on-ubuntu-linux/)).

2\. The downloaded installer zip file should be in your `Downloads` folder. You can navigate to this
folder by entering the following command in your terminal:

```sh
cd ~/Downloads
```

3\. You should now be able to see the installer `.zip` file, if you run the following command:

```sh
ls
```

4\. To unzip the installer, run the following command (this may take a second):

```sh
unzip <zip_file_name>
```

For example:

```sh
unzip stm32cubemx-lin-v6-16-0.zip
```

5\. You should now be able to see the installer executable along with `Readme.html` and a Java runtime by running the following command:

```sh
ls
```

6\. Run the installer by running the following (you may be prompted to enter your password):

```sh
sudo ./SetupSTM32CubeMX-<version>
```

For example:

```sh
sudo ./SetupSTM32CubeMX-6.16.0
```

7\. In the UI that opens, select `Next` and accept the License and Terms of Use.

8\. Select an appropriate installation directory (we recommend `/usr/local/STMicroelectronics/STM32Cube/STM32CubeMX`)

9\. Once the installation completes, select `Done`.

10\. To clean the installation, remove all files in `~/Downloads` from the installer with the following:

```sh
rm -rf jre Readme.html SetupSTM32CubeMX* stm32cubemx-lin-*.zip
```

11\. To use STM32CubeMX from the command line, it must be first added to the system path. To do this, we will use `vi` ([guide](https://opensource.com/article/19/3/getting-started-vim)). Run the following command:

```sh
sudo vi /etc/environment
```

And append your installation directory (for example, `/usr/local/STMicroelectronics/STM32Cube/STM32CubeMX`) to `PATH`, separated from the other paths by a `:`.

12\. Reboot your machine, open a terminal, and verify that the following command can find STM32CubeMX (it should output the path used in step 11):

```sh
which STM32CubeMX
```

## Downloading and Installing CubeCLT

### 1. Download CubeCLT

1. Go to the CubeCLT [download page](https://www.st.com/en/development-tools/stm32cubeclt.html) and scroll down to "Get Software."
   ![get cubeclt](get-cubeclt.webp)

2. **_For this walkthrough, we will be using the Debian Linux installer for UBUNTU USERS._**
   Click on "Select Version", then select the **newest version** for the **_correct OS you are running!!!_**

3. Click on Accept for the License Agreement pop up.

4. You will then be prompted to log in, create an account, or continue as a guest. You will need a
   MyST account in the future, so it is best to create one now. You may use any email address.

5. After logging in and being brought back to the download page, the download should start automatically.
   Scroll down and select the correct version from "Get Software" if it does not.

### 2. Install CubeCLT

**Note**: This install guide is for **Debian Linux**. Please make sure to install the correct version
for the OS you are running.

1\. Open a new terminal ([guide](https://www.howtogeek.com/686955/how-to-launch-a-terminal-window-on-ubuntu-linux/)).

2\. The downloaded installer zip file should be in your `Downloads` folder. You can navigate to this
folder by entering the following command in your terminal:

```sh
cd ~/Downloads
```

3\. You should now be able to see the installer `.zip` file, if you run the following command:

```sh
ls
```

4\. To unzip the installer, run the following command (this may take a second):

```sh
unzip <zip_file_name>
```

For example:

```sh
unzip st-stm32cubeclt_1.20.0_26822_20251117_1245_amd64.deb_bundle.sh.zip
```

5\. You should now be able to see the installer script by running the following command:

```sh
ls
```

6\. Run the installer by running the following (you may be prompted to enter your password):

```sh
sudo chomd +x <script>
sudo ./<script>
```

For example:

```sh
sudo chmod +x st-stm32cubeclt_1.20.0_26822_20251117_1245_amd64.deb_bundle.sh
sudo ./st-stm32cubeclt_1.20.0_26822_20251117_1245_amd64.deb_bundle.sh
```

7\. Accept the License Agreement (`y` and then `Enter`).

8\. To clean the installation, remove all files in `~/Downloads` from the installer with the following:

```sh
rm st-stm32cubeclt*
```

9\. To be able to use the various STM32CubeCLT components, add them all to path as was done with CubeMX.

The install path to STM32CubeCLT is `/opt/st/`, but each tool included has its own `bin` directory that must be added
to the path independently.

The tools to be added are as follows.

- GNU-tools-for-STM32
- st-arm-clang
- STLink-gdb-server
- STM32CubeProgrammer

## CubeMX and CubeCLT for macOS Users

### 1. Download

Download [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html) and [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html)
for macOS. By default your `Downloads` folder should now contain two [tar](<https://en.wikipedia.org/wiki/Tar_(computing)>) files,
one for each tool.

### 2. Install

Double click on each tar file. One tar file will expand into a `.app` bundle: double click this item
and follow the installation steps. The other tar file should expand into a folder with two .pkg files:
click on each one respectively and follow the steps for installation.

### 3. Update your PATH

The `scripts/new.sh` script (used when [creating a new project](#creating-a-new-project)) needs to
know where the STM32CubeMX and STM32CubeCLT binaries exist, which we can specify by adding the location
of the binaries to our [`PATH` environment variable](<https://en.wikipedia.org/wiki/PATH_(variable)>).

There are multiple ways to do this, but the way we will choose is to add the paths to the system-wide
`/etc/paths` file:

1. Run the command `sudo nano /etc/paths` to open a text editor in the terminal where we can update this file.

2. Ensure that all the following paths exist in the file. If a path is missing, type it in on a new line.
   Once you are done, make sure you write out (`^O`) and exit (`^X`).
   ![verify macos path](./verify-paths-macos.webp)

3. Restart your terminal. Ensure the paths you added have been added to your PATH environment variable
   by running the command `echo $PATH`. You should see all the paths you added somewhere in the output.

### 4. Next Steps

You should now be able to create a new project following the [steps below](#creating-a-new-project).

## Creating a New Project

This quick guide will teach you how to make a new project for your STM32G431RB Nucleo board that you
will be developing on.

### Prerequisites

- STM32CubeMX and STM32CubeCLT [installed](../stm32cube/index.md)

### Guide

To create a new project, use the `scripts/new.sh` script. The script accepts either an MCU or Development Board ID, project source, and optionally any number of cmake libraries defined under `lib`. To create a project for the Nucleo G431RB developer kit, run the following.

```bash
./scripts/new.sh --board NUCLEO-G431RB --src <path/to/project>
```

When prompted to select default peripheral configurations, select "Unselect All" and "continue".

If this is the first time STM32CubeMX is being run on a machine, it may need to download the firmware repository. Select "Download" and continue.

Once the script completes, try to build the generated project as follows.

```bash
./scripts/build.sh --src <path/to/project>
```

If this completes successfully, then STM32CubeCLT is correctly installed on the system.

Open the `<project>.ioc` file in STM32CubeMX to modify the project configuration.

**Congratulations! You have successfully created a new project with CubeMX!**

For information on how to do this process manually, refer to the [CMake + CubeMX/CubeCLT Toolchain](../../extra/cmake-cubemx.md) document.
