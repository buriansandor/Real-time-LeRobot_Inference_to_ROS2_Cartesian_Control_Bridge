import re
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_log(filename):
    raw_points = []
    tgt_points = []
    
    with open(filename, 'r') as f:
        for line in f:
            # [RAW] X=... | -> [TGT] X=...
            match = re.search(r'\[RAW\] X=([-\d\.]+) Y=([-\d\.]+) Z=([-\d\.]+).*\[TGT\] X=([-\d\.]+) Y=([-\d\.]+) Z=([-\d\.]+)', line)
            if match:
                raw_points.append([float(match.group(1)), float(match.group(2)), float(match.group(3))])
                tgt_points.append([float(match.group(4)), float(match.group(5)), float(match.group(6))])
                
    return raw_points, tgt_points

raw, tgt = parse_log('follower_log.txt')

fig = plt.figure(figsize=(12, 6))

# Plot RAW (Leader)
ax1 = fig.add_subplot(121, projection='3d')
xs, ys, zs = zip(*raw)
ax1.scatter(xs, ys, zs, c='r', marker='o')
ax1.set_title('Leader Input (RAW)')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# Plot TARGET (Follower)
ax2 = fig.add_subplot(122, projection='3d')
txs, tys, tzs = zip(*tgt)
ax2.scatter(txs, tys, tzs, c='b', marker='^')
ax2.set_title('Follower Target (TGT)')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')

plt.show()