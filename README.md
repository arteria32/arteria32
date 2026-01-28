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

## Mujoco: get joint `qpos` by name

In MuJoCo, generalized positions live in `data.qpos` (length `model.nq`). Each joint occupies a slice of `qpos`, starting at `model.jnt_qposadr[jid]`. The slice length depends on joint type:

- `mjJNT_HINGE`, `mjJNT_SLIDE`: 1
- `mjJNT_BALL`: 4 (unit quaternion)
- `mjJNT_FREE`: 7 (pos(3) + quat(4))

### Python (`mujoco` >= 2.3)

```python
import mujoco
import numpy as np

def joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> np.ndarray:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise KeyError(f"Unknown joint: {joint_name!r}")

    adr = model.jnt_qposadr[jid]
    jtype = model.jnt_type[jid]
    size = {
        mujoco.mjtJoint.mjJNT_HINGE: 1,
        mujoco.mjtJoint.mjJNT_SLIDE: 1,
        mujoco.mjtJoint.mjJNT_BALL: 4,
        mujoco.mjtJoint.mjJNT_FREE: 7,
    }[jtype]
    return data.qpos[adr : adr + size].copy()

# Example:
# q = joint_qpos(model, data, "elbow")
```

If you’re on a newer Python API, you can also do:

```python
# j = model.joint("elbow")
# adr = j.qposadr
# (size still depends on joint type; slice data.qpos[adr:adr+size])
```

### C

```c
#include <mujoco/mujoco.h>

static int joint_qpos_size(mjtJoint jtype) {
  switch (jtype) {
    case mjJNT_HINGE: return 1;
    case mjJNT_SLIDE: return 1;
    case mjJNT_BALL:  return 4;
    case mjJNT_FREE:  return 7;
    default:          return 0;
  }
}

// Writes up to 7 values into out_qpos, returns size; -1 if not found.
int get_joint_qpos(const mjModel* m, const mjData* d, const char* joint_name, mjtNum out_qpos[7]) {
  int jid = mj_name2id(m, mjOBJ_JOINT, joint_name);
  if (jid < 0) return -1;

  int adr = m->jnt_qposadr[jid];
  int sz  = joint_qpos_size((mjtJoint)m->jnt_type[jid]);
  for (int i = 0; i < sz; ++i) out_qpos[i] = d->qpos[adr + i];
  return sz;
}
```

