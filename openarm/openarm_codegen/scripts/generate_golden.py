"""Reference values for the generated OpenArm model, from Pinocchio in double precision (offline).

40 random states of both arms (joint order: left J1..J7, right J1..J7; fingers at 0, as in the
generated model). Each row: q(14) dq(14) ddq(14) M(196, row-major) C(196, row-major) G(14)
tau_id(14) where tau_id = M ddq + C dq + G (inverse dynamics, RNEA).

    python3 generate_golden.py <official urdf> <output csv>
"""
import sys

import numpy as np
import pinocchio as pin

urdf, out = sys.argv[1], sys.argv[2]
model = pin.buildModelFromUrdf(urdf)
data = model.createData()
names = [f"openarm_{s}_joint{j}" for s in ("left", "right") for j in range(1, 8)]
iq = [model.joints[model.getJointId(n)].idx_q for n in names]
iv = [model.joints[model.getJointId(n)].idx_v for n in names]
lo, hi = model.lowerPositionLimit[iq], model.upperPositionLimit[iq]
rng = np.random.default_rng(20261009)
rows = []
for _ in range(40):
    q, dq, ddq = rng.uniform(lo, hi), rng.uniform(-1, 1, 14), rng.uniform(-3, 3, 14)
    Q, V, A = pin.neutral(model), np.zeros(model.nv), np.zeros(model.nv)
    Q[iq], V[iv], A[iv] = q, dq, ddq
    M = pin.crba(model, data, Q)
    M = np.triu(M) + np.triu(M, 1).T
    C = pin.computeCoriolisMatrix(model, data, Q, V)
    G = pin.computeGeneralizedGravity(model, data, Q)
    tau = pin.rnea(model, data, Q, V, A)
    sel = np.ix_(iv, iv)
    rows.append(np.concatenate([q, dq, ddq, M[sel].ravel(), C[sel].ravel(), G[iv], tau[iv]]))
header = "# joints: " + " ".join(names) + "\n# q dq ddq M(row-major) C(row-major) G tau_id ; fingers at 0\n"
with open(out, "w") as f:
    f.write(header)
    for r in rows:
        f.write(" ".join(format(v, ".17g") for v in r) + "\n")
print(f"{len(rows)} cases -> {out}")
