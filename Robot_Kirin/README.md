# Robot_Kirin – AdvantageScope Custom Robot Asset

This folder contains our custom robot model with articulated parts.

Robot_Kirin

Now follow the steps below to make it work.

---

## 🛠 Step 1 – Open AdvantageScope Assets Folder

1. Open AdvantageScope
2. Click:
   App → AdvantageScope → Show Assets Folder
3. A folder will open on your computer

This is the root assets directory.

---

## 📂 Step 2 – Drag the Folder

Drag your entire:

Robot_Kirin

folder into the opened AdvantageScope assets folder.

Do NOT drag only the files.
Drag the entire Robot_Kirin folder.

The final structure should look like:

The final structure should look like:

```
AdvantageScopeAssets/
└── Robot_Kirin/
    ├── model.glb
    ├── ...
    └── config.json
```

---

## 🔁 Step 3 – Restart AdvantageScope

AdvantageScope should now reload.

Your robot should now appear in the robot selection dropdown as:

Kirin

---

## Setting up

Now that you have the robot model selected, you need to add these two fields. For now this only articulates in sim.

Drag and drop the fields below ontop of the Robot component in AdvantageScope

```
NT:/AdvantageKit/RealOutputs/Superstructure/ComponentPoses
NT:/AdvantageKit/RealOutputs/Intake/ComponentPoses

Please ensure you do it in this order!
```

![AdvantageScope](image.png)
