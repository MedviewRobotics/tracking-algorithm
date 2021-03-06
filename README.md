[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Issues][issues-shield]][issues-url]


<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/MedviewRobotics/tracking-algorithm">
    <img src="images/Logo.png" alt="Logo" width="225" height="240">
  </a>

  <h3 align="center">Robotic Automated Microscopy</h3>

  <p align="center">
    This robotic automated microscopy system aims to improve surgical safety through the tracking of a surgical instrument throughout surgery. Given the location of the surgical instrument provided by the CV tracking algorithm, the control system will adjust the position of a microscope at the end-effector of a robotic arm accordingly to provide a zoomed-in view of the surgical field.
    <br />
    <a href="https://github.com/MedviewRobotics/tracking-algorithm/issues">Report Bug</a>
    ·
    <a href="https://github.com/MedviewRobotics/tracking-algorithm/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary><h2 style="display: inline-block">Table of Contents</h2></summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgements">Acknowledgements</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

</br>
<p align="center">
  <a href="https://github.com/MedviewRobotics/tracking-algorithm">
    <img src="images/Assembly of Components v15 (2).gif" alt="Animation of Project" width="640" height="271">
  </a>
</p>
</br>
Our team designed a Robotic Automated Microscopy System for our final capstone design project for the Department of Electrical, Computer, and Biomedical Engineering at Ryerson University. This system is designed to improve the accuracy and safety of surgical procedures, and maintain an ergonomic experience for the OR staff. The parameters of this design challenge were to accomplish this task with a minimum of two degrees of freedom (DOF), a control system operating at a minimum of 10Hz, a tracking system operating at a minimum of 20FPS, and have the entire system adjust to a surgeon’s movements in as close to real-time as possible.
</br>
</br>
The high-level workflow of our system can be visualized as follows:
</br>
<p align="center">
  <a href="https://github.com/MedviewRobotics/tracking-algorithm">
    <img src="images/SysOverview.png" alt="Workflow" width="667" height="251">
  </a>
</p> 
</br>
The system in this repository includes an object tracking algorithm as well as the robotic control system. The robotic structure is a 6DOF anthropomorphic arm carrying a microscope camera weighing approximately 55g at the end-effector. The .STL files of our custom-designed robot have been imported to the 3D MATLAB plot for visualization. The tracking algorithm comprises several object tracking steps and is able to process at a rate of 16 FPS. The designed control system achieves joint trajectory angles needed to actuate the robotic arm at a frequency rate of 29 Hz. Results indicate average accuracies of 87.2% for the designed tracking algorithm and 92.3% for our control system. The end-to-end system is able to successfully track a surgeon's surgical instrument movement and keep the microscope end-effector pointed at the surgical field of view for real time visualization on external TV monitors.
</br>
</br>
<p align="center">
  <a href="https://github.com/MedviewRobotics/tracking-algorithm">
    <img src="images/Control_System_Plot.png" alt="Control System Plot" width="314" height="225">
  </a>
  <a href="https://github.com/MedviewRobotics/tracking-algorithm">
    <img src="images/Optical_Tracking_System_Plot.png" alt="Optical Tracking System Plot" width="407" height="225">
  </a>
</p> 
</br>


### Built With

* [MATLAB2020a](https://www.mathworks.com/products/new_products/release2020a.html?s_tid=srchtitle)
* [Peter Corke Robotic's Toolbox](https://petercorke.com/toolboxes/robotics-toolbox/)


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Pre-Requisites

* [MATLAB2020a](https://www.mathworks.com/products/new_products/release2020a.html?s_tid=srchtitle)
* [MATLAB Computer Vision Toolbox](https://www.mathworks.com/help/vision/index.html?searchHighlight=Computer%20vision&s_tid=srchtitle) 
* [MATLAB Image Processing Toolbox](https://www.mathworks.com/products/image.html?s_tid=srchtitle)
* [MATLAB Communications Toolbox](https://www.mathworks.com/help/comm/index.html?s_tid=srchtitle)
* [MATLAB Partial Differential Equations Toolbox](https://www.mathworks.com/products/pde.html#:~:text=Partial%20Differential%20Equation%20Toolbox%20lets,to%20explore%20and%20analyze%20them.)

### Installation

1. Install pre-requisites via links provided
2. Clone the repo
   ```sh
   git clone https://github.com/MedviewRobotics/tracking-algorithm.git
   ```

<!-- USAGE EXAMPLES -->
## Usage

The entire system can be ran from the [Main.m](https://github.com/MedviewRobotics/tracking-algorithm/blob/main/Accuracy%20and%20Performance/Main.m) file in the Accuracy and Performance folder. By default the system will read in one of the trial videos that we have prepared for demonstration purposes.
</br>
</br>
A full demonstration of the code and system overview can be found [here!](https://drive.google.com/file/d/1_XPp7Msm0ABWXXz6eFeGKVfyWz81GJaR/view?usp=sharing)

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request


<!-- CONTACT -->
## Contact

1. Claudia Alonzo - [LinkedIn](https://www.linkedin.com/in/claudia-alonzo098/) - claudia.alonzo@ryerson.ca
2. Ginette Hartell - [LinkedIn](https://www.linkedin.com/in/ginette-hartell/) - ghartell@ryerson.ca
3. Jay Tailor - [LinkedIn](https://www.linkedin.com/in/tailor-jay/) - jay.tailor@ryerson.ca
4. Aziz Uddin - [LinkedIn](https://www.linkedin.com/in/azizuddin1234/) - aziz.uddin@ryerson.ca

<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

First and foremost, we would like to express our deep and sincere gratitude to our faculty advisor Dr. Ali Tavallaei for his outstanding commitment to our success in this project. Dr. Tavallaei has been patient and thorough when explaining new concepts to us and has been an expert at tying these new concepts back to theory that we have learned in our previous courses. He has challenged us to make design decisions based on data and solid justifications, which has helped us deepen our learning as a team.

We would also like to extend our appreciation to our Biomedical Image Analysis Professor, Dr. April Khademi. Dr. Khademi has been instrumental in our learning of the core concepts of image processing, and was gracious enough to extend her mentorship beyond the field of medical image processing to help us evaluate the effectiveness of our object tracking algorithm.

Additionally, we would like to sincerely thank Josh Richmond of Synaptive Medical for his insight into the challenges and intricacies of the surgical system navigation industry. His emphasis on an iterative approach to product development was extremely helpful throughout the course of this project.

Lastly, we wanted to thank Peter Corke, the creator of the Robotics Toolbox and the author of the “Robotics, Vision & Control”. The open-source toolbox provided many functions that were useful for the study and simulation of classical arm-type robotics. It was critical when performing kinematics, and trajectory generation.


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/MedviewRobotics/tracking-algorithm.svg?style=for-the-badge
[contributors-url]: https://github.com/MedviewRobotics/tracking-algorithm/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/MedviewRobotics/tracking-algorithm.svg?style=for-the-badge
[forks-url]: https://github.com/MedviewRobotics/tracking-algorithm/network/members
[issues-shield]: https://img.shields.io/github/issues/MedviewRobotics/tracking-algorithm.svg?style=for-the-badge
[issues-url]: https://github.com/MedviewRobotics/tracking-algorithm/issues
