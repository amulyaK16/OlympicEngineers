import React, { Component, useState, useRef, useEffect } from "react";
import { Link } from "react-router-dom";
import { FormControlLabel } from "@material-ui/core/FormControlLabel";
import {
  TextField,
  Button,
  Grid,
  Typography,
  FormHelperText,
  FormControl,
  Radio,
  RadioGroup,
  Card,
  CardHeader,
  CardContent,
  IconButton,
} from "@material-ui/core";
import { Chart as ChartJS } from "chart.js";
import { Line, Bar } from "react-chartjs-2";
import "chartjs-plugin-streaming";
import moment from "moment";
import MoreVertIcon from "@material-ui/icons/MoreVert";

async function fetchData(datatype, code) {
  var api_addr = "http://127.0.0.1:8000/api/GetData?data="
    .concat(datatype)
    .concat("&username=")
    .concat(code);
  const requestOptions = {
    method: "GET",
    headers: { "Content-Type": "application/json" },
  };
  const result = await fetch(api_addr, requestOptions)
    .then((response) => response.json())
    .then((data) => {
      console.log("retrieved data: ");
      console.log(data.data);
      return data.data;
    })
    .catch((error) => {
      console.log(error);
    });
  return result;
}

function ECGChart() {
  const chartRef = useRef(null);

  useEffect(() => {
    const chart = chartRef.current;
    if (chart) {
      console.log("ecg chart:", chart);
    }
    const interval = setInterval(() => {
      console.log("current ecg data: ");
      console.log(data.datasets[0].data);
    }, 5000);
  }, []);

  const data = {
    datasets: [
      {
        label: "ECG Data",
        borderColor: "rgb(255, 99, 132)",
        backgroundColor: "rgba(255, 99, 132, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      },
    ],
  };

  const options = {
    scales: {
      xAxes: [
        {
          type: "realtime",
          realtime: {
            refresh: 2000,
            onRefresh: function () {
              var api_addr = "http://127.0.0.1:8000/api/GetECG?username="
                .concat("haydnbrown");
              const requestOptions = {
                method: "GET",
                headers: { "Content-Type": "application/json" },
              };

              fetch(api_addr, requestOptions)
                .then((response) => response.json())
                .then((data) => {
                  console.log("retrieved ecg data: ");
                  console.log(data.data);
                  chartRef.current.chartInstance.data.datasets[0].data.push({
                    x: Date.now(),
                    y: data.data,
                  });
                  chartRef.current.chartInstance.update();
                })
                .catch((error) => {
                  console.log(error);
                });
              
            },
            delay: 2000,
            frameRate: 15,
          },
        },
      ],
    },
  };

  return (
    <div>
      <Line data={data} options={options} ref={chartRef} />
    </div>
  );
}

function EMGChart() {
  const chartRef = useRef(null);

  useEffect(() => {
    const chart = chartRef.current;
    if (chart) {
      console.log("emg chart:", chart);
    }
    const interval = setInterval(() => {
      console.log("current emg data: ");
      console.log(data.datasets[0].data);
    }, 5000);
  }, []);

  const data = {
    datasets: [
      {
        label: "EMG Data",
        borderColor: "rgb(255, 99, 132)",
        backgroundColor: "rgba(255, 99, 132, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      },
    ],
  };

  const options = {
    scales: {
      xAxes: [
        {
          type: "realtime",
          realtime: {
            refresh: 2000,
            onRefresh: function () {
              var api_addr = "http://127.0.0.1:8000/api/GetEMG?username="
                .concat("haydnbrown");
              const requestOptions = {
                method: "GET",
                headers: { "Content-Type": "application/json" },
              };

              fetch(api_addr, requestOptions)
                .then((response) => response.json())
                .then((data) => {
                  console.log("retrieved emg data: ");
                  console.log(data.data);
                  chartRef.current.chartInstance.data.datasets[0].data.push({
                    x: Date.now(),
                    y: data.data,
                  });
                  chartRef.current.chartInstance.update();
                })
                .catch((error) => {
                  console.log(error);
                });
              
            },
            delay: 2000,
            frameRate: 15,
          },
        },
      ],
    },
  };

  return (
    <div>
      <Line data={data} options={options} ref={chartRef} />
    </div>
  );
}

function ForceChart() {
  const chartRef = useRef(null);

  useEffect(() => {
    const chart = chartRef.current;
    if (chart) {
      console.log("force chart:", chart);
    }
    const interval = setInterval(() => {
      console.log("current force data: ");
      console.log(data.datasets[0].data);
    }, 5000);
  }, []);

  const data = {
    datasets: [
      {
        label: "Force Data",
        borderColor: "rgb(255, 99, 132)",
        backgroundColor: "rgba(255, 99, 132, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      },
    ],
  };

  const options = {
    scales: {
      xAxes: [
        {
          type: "realtime",
          realtime: {
            refresh: 2000,
            onRefresh: function () {
              var api_addr = "http://127.0.0.1:8000/api/GetForce?username="
                .concat("haydnbrown");
              const requestOptions = {
                method: "GET",
                headers: { "Content-Type": "application/json" },
              };

              fetch(api_addr, requestOptions)
                .then((response) => response.json())
                .then((data) => {
                  console.log("retrieved force data: ");
                  console.log(data.data);
                  chartRef.current.chartInstance.data.datasets[0].data.push({
                    x: Date.now(),
                    y: data.data,
                  });
                  chartRef.current.chartInstance.update();
                })
                .catch((error) => {
                  console.log(error);
                });
              
            },
            delay: 2000,
            frameRate: 15,
          },
        },
      ],
    },
  };

  return (
    <div>
      <Line data={data} options={options} ref={chartRef} />
    </div>
  );
}

function GyroChart() {
  const chartRef = useRef(null);

  useEffect(() => {
    const chart = chartRef.current;
    if (chart) {
      console.log("gyro chart:", chart);
    }
    const interval = setInterval(() => {
      console.log("current gyro data: ");
      console.log(data.datasets[0].data);
    }, 5000);
  }, []);

  const data = {
    datasets: [
      {
        label: "Gyro x axis",
        borderColor: "rgb(255, 99, 132)",
        backgroundColor: "rgba(255, 99, 132, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      }, {
        label: "Gyro y axis",
        borderColor: "rgb(154, 17, 189)",
        backgroundColor: "rgba(154, 17, 189, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      }, {
        label: "Gyro z axis",
        borderColor: "rgb(140, 77, 8)",
        backgroundColor: "rgba(140, 77, 8, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      },
    ],
  };

  const options = {
    scales: {
      xAxes: [
        {
          type: "realtime",
          realtime: {
            refresh: 2000,
            onRefresh: function () {
              var api_addr = "http://127.0.0.1:8000/api/GetGyro?username="
                .concat("haydnbrown");
              const requestOptions = {
                method: "GET",
                headers: { "Content-Type": "application/json" },
              };

              fetch(api_addr, requestOptions)
                .then((response) => response.json())
                .then((data) => {
                  console.log("retrieved gyro data: ");
                  console.log(data);
                  chartRef.current.chartInstance.data.datasets[0].data.push({
                    x: Date.now(),
                    y: data.x,
                  });
                  chartRef.current.chartInstance.data.datasets[1].data.push({
                    x: Date.now(),
                    y: data.y,
                  });
                  chartRef.current.chartInstance.data.datasets[2].data.push({
                    x: Date.now(),
                    y: data.z,
                  });
                  chartRef.current.chartInstance.update();
                })
                .catch((error) => {
                  console.log(error);
                });
              
            },
            delay: 2000,
            frameRate: 15,
          },
        },
      ],
    },
  };

  return (
    <div>
      <Line data={data} options={options} ref={chartRef} />
    </div>
  );
}

function AccelChart() {
  const chartRef = useRef(null);

  useEffect(() => {
    const chart = chartRef.current;
    if (chart) {
      console.log("accel chart:", chart);
    }
    const interval = setInterval(() => {
      console.log("current accel data: ");
      console.log(data.datasets[0].data);
    }, 5000);
  }, []);

  const data = {
    datasets: [
      {
        label: "Accelerometer x axis",
        borderColor: "rgb(255, 99, 132)",
        backgroundColor: "rgba(255, 99, 132, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      }, {
        label: "Accelerometer y axis",
        borderColor: "rgb(154, 17, 189)",
        backgroundColor: "rgba(154, 17, 189, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      }, {
        label: "Accelerometer z axis",
        borderColor: "rgb(140, 77, 8)",
        backgroundColor: "rgba(140, 77, 8, 0.5)",
        lineTension: 0,
        borderDash: [8, 4],
        data: [],
      },
    ],
  };

  const options = {
    scales: {
      xAxes: [
        {
          type: "realtime",
          realtime: {
            refresh: 2000,
            onRefresh: function () {
              var api_addr = "http://127.0.0.1:8000/api/GetAccel?username="
                .concat("haydnbrown");
              const requestOptions = {
                method: "GET",
                headers: { "Content-Type": "application/json" },
              };

              fetch(api_addr, requestOptions)
                .then((response) => response.json())
                .then((data) => {
                  console.log("retrieved Accelerometer data: ");
                  console.log(data);
                  chartRef.current.chartInstance.data.datasets[0].data.push({
                    x: Date.now(),
                    y: data.x,
                  });
                  chartRef.current.chartInstance.data.datasets[1].data.push({
                    x: Date.now(),
                    y: data.y,
                  });
                  chartRef.current.chartInstance.data.datasets[2].data.push({
                    x: Date.now(),
                    y: data.z,
                  });
                  chartRef.current.chartInstance.update();
                })
                .catch((error) => {
                  console.log(error);
                });
              
            },
            delay: 2000,
            frameRate: 15,
          },
        },
      ],
    },
  };

  return (
    <div>
      <Line data={data} options={options} ref={chartRef} />
    </div>
  );
}


function Greeting(props) {
  return <p>{props.value}, Haydn...</p>;
}

export default function SignedInWrapper() {
  return (
    <Grid container spacing={1}>
      <Grid item sm={12} align="center">
        <h1>Live Data Panel</h1>
      </Grid>
      <Grid item xs={4}>
        <Card elevation={15}>
          <CardHeader title={<Typography variant="h5">ECG sensor data</Typography>} />
          <CardContent style={{ backgroundColor: "#fff" }}>
            <ECGChart />
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={4}>
        <Card elevation={15}>
          <CardHeader title={<Typography variant="h5">EMG sensor data</Typography>} />
          <CardContent style={{ backgroundColor: "#fff" }}>
            <EMGChart />
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={4}>
        <Card elevation={15}>
          <CardHeader title={<Typography variant="h5">Force sensor data</Typography>} />
          <CardContent style={{ backgroundColor: "#fff" }}>
            <ForceChart />
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={6}>
        <Card elevation={15}>
          <CardHeader title={<Typography variant="h5">Gyro sensor data</Typography>} />
          <CardContent style={{ backgroundColor: "#fff" }}>
            <GyroChart />
          </CardContent>
        </Card>
      </Grid>
      <Grid item xs={6}>
        <Card elevation={15}>
          <CardHeader title={<Typography variant="h5">Accelerometer sensor data</Typography>} />
          <CardContent style={{ backgroundColor: "#fff" }}>
            <AccelChart />
          </CardContent>
        </Card>
      </Grid>
    </Grid>
  );
}
