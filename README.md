# Fundamental Matrix Estimation from Two Views

This repository contains a Python implementation of **Fundamental Matrix estimation** for 3D objects, as part of a 3D reconstruction course. The project demonstrates the projection of 3D points from an OBJ file into two camera views and estimates the fundamental matrix between these views using 2D correspondences.

The pipeline includes:
- Loading 3D points from `.obj` files.
- Defining camera intrinsic and extrinsic parameters.
- Projecting 3D points to 2D image planes.
- Normalizing 2D points for numerical stability.
- Estimating the fundamental matrix from point correspondences.

---

# Installation

Clone the repository:

```
git clone [https://github.com/Baya-Mezghani/3D-Fundamental-Matrix.git]
cd 3D-Fundamental-Matrix
```

---

# Create a Virtual Environment

Create a Python virtual environment:

```
python -m venv venv
```

Activate the environment.

```
venv\Scripts\activate
```

---

# Install Dependencies

Install the required packages:

```
pip install -r requirements.txt
```

---

# Run the Project

Run the main script:

```
python code/main.py
```
The program will:

*Load and normalize the 3D points.
*Project them into two different camera views.
*Select random 2D correspondences.
*Estimate and print the Fundamental Matrix.

---
# Project Structure

```
3D-Fundamental-Matrix
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── code
│   └── main.py
│
└── data
    └── wooden_chair.obj
```
---

# Technologies Used

* Python
* NumPy
* Matplotlib

---

# Author

**Baya Mezghani** 
📧 baya.mezghani@ensi-uma.tn
