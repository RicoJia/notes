fig, ax = plt.subplots(figsize=(5, 4))

x, y = frame["pressure (pa)"], frame["ground_truth_height (m)"]
ax.scatter(x, y, s=0.5)
ax.set_ylabel("ground truth height (m)")
ax.set_xlabel("pressure delta (unit?)")
slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(x, y)
x1 = np.linspace(min(x), max(x), 500)
y1 = slope*x1 + intercept

r_str = f"$R^2$: {r_value ** 2:0.4f}"
slope_str = f"slope: {slope:0.6f}"
intercept_str = f"intercept: {intercept:0.6f}"

print(r_str)
print(slope_str)
print(intercept_str)

ax.plot(x1, y1, color="red")

ax.text(0.1, 0.9, r_str, fontsize=10, horizontalalignment="left", verticalalignment="center", transform=ax.transAxes)
ax.text(0.1, 0.85, slope_str, fontsize=10, horizontalalignment="left", verticalalignment="center", transform=ax.transAxes)
ax.text(0.1, 0.8, intercept_str, fontsize=10, horizontalalignment="left", verticalalignment="center", transform=ax.transAxes)
