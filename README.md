# WiFiSolarVoltageMonitor
A ESP8266 NodeMCU based IoT to monitor voltage and current of solar power system via WiFi.


## Setting up InfluxDB Time-series  Database

This mostly follows the instructions given here:
[https://hub.docker.com/_/influxdb](https://hub.docker.com/_/influxdb)

Run the server to dump a default config file and then exit
~~~bash
mkdir -p ${HOME}/HomeSolar/etc/influxdb2
docker run --rm influxdb:2.7.1 influxd print-config \
    > ${HOME}/HomeSolar/etc/influxdb2/config.yml
~~~

Modify the config. file if needed then run the server again, mounting
the appropriate directories from the host and forwarding the 8086 port.
~~~bash
mkdir -p ${HOME}/HomeSolar/DB
docker run -d -p 8086:8086 \
    --name influxdb2 --rm \
    -v ${HOME}/HomeSolar/etc/influxdb2:/etc/influxdb2 \
    -v ${HOME}/HomeSolar/DB:/var/lib/influxdb2 \
    influxdb:2.7.1
~~~

Do the initial server setup, creating a user and organization.
~~~bash
export USERNAME=HLSolar
export PASSWORD=ABCDEF1234  # <set to something better!>
export ORGANIZATION=Hill-Lawrence
docker exec influxdb2 influx setup \
    --username $USERNAME \
    --password $PASSWORD \
    --org $ORGANIZATION \
    --bucket default_bucket \
    --force
~~~

Create a token for accessing the DB. The "token" part of this needs to be copied into the `WiFiSolarVoltageMonitor.py` script.
~~~bash
docker exec influxdb2 influx auth create --org $ORGANIZATION --all-access
~~~

Add a "Bucket" (i.e database) with a 100year retention policy. The default retention policy is 30days, but we don't ever need to delete this data.
~~~bash
export BUCKET=HL_Solar
docker exec influxdb2 influx bucket create -n $BUCKET -o $ORGANIZATION -r 5200w
~~~

## Running the `WiFiSolarVoltageMonitor.py` script
This script needs to be edited to update the `vmon_url` and the `db_token` at the very least. Once this is done, it can be started with no arguments. It should print a reading every 10 seconds when it makes a new DB entry. 
(Place in a crontab to autimatically start on reboot if desired.)


## Setting up a dashboard.

The InfluxDB instance has a built-in dashboard that looks a lot like Grafana (and
may even *be* Grafana under the hood). Just point a web browser to the `8086` port on the host running the server. For example:

[http://localhost:8086]()

Use the username/password specified above to log in and create a dashboard. Assuming the device has entered at least one data point, you should be able
to see the fields and construct the dashboard.
