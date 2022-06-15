from statsd import StatsClient
stats_client = StatsClient()
timer=stats_client.timer('statsd')
timer.start()
timer.stop()

