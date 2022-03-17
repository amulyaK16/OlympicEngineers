import React, { Component, useState } from "react";
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
  IconButton
} from "@material-ui/core";
import { Line } from "react-chartjs-2";
import "chartjs-plugin-streaming";
import moment from "moment";
import { Bar } from "react-chartjs-2";
import MoreVertIcon from "@material-ui/icons/MoreVert";

function Chart1(){
    const data = {
        datasets: [
          {
            label: "Dataset 1",
            borderColor: "rgb(255, 99, 132)",
            backgroundColor: "rgba(255, 99, 132, 0.5)",
            lineTension: 0,
            borderDash: [8, 4],
            data: []
          }
        ]
      };
    
      const options = {
        scales: {
          xAxes: [
            {
              type: "realtime",
              realtime: {
                onRefresh: function() {
                  data.datasets[0].data.push({
                    x: Date.now(),
                    y: Math.random() * 100
                  });
                },
                delay: 2000
              }
            }
          ]
        }
      };
    
      return (
      <div>
        <Line data={data} options={options} />
      </div>);
}


function Greeting(props) {
  return <p>{props.value}, Haydn...</p>;
}

export default function SignedInWrapper() {
  return (
    <Grid container spacing={1}>
      <Grid item xs={12} align="center">
        <Greeting value={"Hello"} />
      </Grid>
      <Grid item xs={12} align="center">
        <Card elevation={10}>
          <CardHeader
            title={
              <Typography variant="h5">
                Graph 1
              </Typography>
            }
          />
          <CardContent style={{ backgroundColor: "#fff" }}>
            <Chart1 />
          </CardContent>
        </Card>
      </Grid>
    </Grid>
  );
}
