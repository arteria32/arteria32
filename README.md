<h1 align="center">
Hello, I'm Denis Ryabko
	<a href="https://github.com/Bouaskaoun" target="_self">
		<img src="https://media.giphy.com/media/hvRJCLFzcasrR4ia7z/giphy.gif" width="30">
	</a>
</h1>

<hr>

<pre>
💻 I am mainly a Frontend Developer
📚 I have a Bachelors in Gas Reservoir Engineering from the Gubkin University
📝 I have a strong interest in Frontend and geomechanical modeling 
🛠️ Currently working on a Gazprom Neft
🌟 Main language: JavaScript
🚩 Interested in learning more about Microfrontend Architectures.
😃 I look forward to collaborate on impactful projects
</pre>
<hr>

## 🤝 Connect with me

<p align="center">
 <a href="www.linkedin.com/in/denis-ryabko-a02543298">
    <img src="https://img.shields.io/badge/LinkedIn-blue?style=for-the-badge&logo=linkedin&logoColor=white" alt="LinkedIn Badge"/>
  </a>
</p>

## 💻 My Tech Stack

<div align="center">
  <img src="https://github.com/devicons/devicon/blob/master/icons/react/react-original-wordmark.svg" title="React" alt="React" width="40" height="40"/>&nbsp;
    <img src="https://github.com/devicons/devicon/blob/master/icons/redux/redux-original.svg" title="Redux" alt="Redux " width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/angular/angular-original.svg" title="Angular" alt="Angular" width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/materialui/materialui-original.svg" title="Material UI" alt="Material UI" width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/nodejs/nodejs-original-wordmark.svg" title="NodeJS" alt="NodeJS" width="40" height="40"/>&nbsp;
   <img src="https://github.com/devicons/devicon/blob/master/icons/go/go-original.svg" title="Golang" alt="Golang" width="40" height="40"/>&nbsp;
   <img src="https://github.com/devicons/devicon/blob/master/icons/python/python-original.svg" title="Python" alt="Python" width="40" height="40"/>&nbsp;
  <img src="https://github.com/devicons/devicon/blob/master/icons/git/git-original-wordmark.svg" title="Git" **alt="Git" width="40" height="40"/>
</div>

<hr>

## MuJoCo tabletop stove contact physics example

This repo includes `stove_table_physics.xml`, a minimal example showing:

- A **wooden table** (fixed)
- A **movable portable stove** (free joint, mass \(=1.5\) kg, realistic friction)
- A **knob** mounted on the stove via a hinge joint (forces/torques applied to the knob transmit to the stove)

Contact filtering uses a simple bitmask scheme:

- Table geom: `contype=1`, `conaffinity=2`
- Stove base geom: `contype=2`, `conaffinity=1`
- Knob collision proxies: `contype=4`, `conaffinity=0` (disabled by default; enable by setting your gripper/hand geom `conaffinity` to include bit `4` and `contype!=0`)

