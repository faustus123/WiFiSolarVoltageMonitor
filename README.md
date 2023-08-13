# WiFiSolarVoltageMonitor
A ESP8266 NodeMCU based IoT to monitor voltage and current of solar power system via WiFi.


## Recording to Time-series InfluxDB Database

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

Create a token for accessing the DB
~~~bash
docker exec influxdb2 influx auth create --org $ORGANIZATION --all-access
~~~

Add a "Bucket" (i.e database) with a 100year retention policy.
~~~bash
export BUCKET=HL_Solar
docker exec influxdb2 influx bucket create -n $BUCKET -o $ORGANIZATION -r 5200w
~~~


### Setting up Grafana server

Create directory to hold grafana configs and fire up server
~~~bash
mkdir -p $HOME/HomeSolar/grafana
docker run -d -p 3000:3000 --name=grafana --rm \
  -v ${HOME}/HomeSolar/grafana:/var/lib/grafana \
  grafana/grafana-enterprise
~~~

