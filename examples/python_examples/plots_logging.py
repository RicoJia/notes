def test_plot():
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

def test_logging():
    import logging
    import os
    # os.chdir("./") # 日志写入地址
    # only the first basicConfig call is effective. Subsequent calls are not.
    # also the log file NEED to exist already
    logging.basicConfig(filename='/tmp/example.log', level=logging.DEBUG, format='%(levelname)s:%(message)s | %(asctime)s') 
    # 注意：上面level设置的是显示的最低严重级别，小于level设置的最低严重级别将不会打印出来
    logging.debug('This message should go to the log file')
    logging.info('So should this')
    logging.warning('And this, too')
    logging.error('And non-ASCII stuff, too, like Øresund and Malmö')

    """
    Ref: https://zhuanlan.zhihu.com/p/476549020
    1. Handler is to direct logs to different places: console (StreamHandler), log files (FileHandler), or email. 
    2. Each handler should set its own severity level.
    """
if __name__ == "__main__":
    test_logging()
