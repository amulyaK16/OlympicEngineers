import React, { Component } from 'react';
import { Link} from "react-router-dom";
import { FormControlLabel } from '@material-ui/core/FormControlLabel';
import { TextField, Button, Grid, Typography, FormHelperText, FormControl, Radio, RadioGroup} from "@material-ui/core";
import { Line } from 'react-chartjs-2';

export default class SignedInPage extends Component {
    constructor(props) {
        super(props);
        this.state = {
            datafirst: {
                labels: ["1","2","3","4","5","6","7","8","9","10"],
                datasets: {
                    label: 'set 1',
                    data: [14,15,13,17,19,21,22,20,18,16],
                    borderColor: 'rgb(255, 99, 132)',
                    backgroundColor: 'rgba(255, 99, 132, 0.5)',
                }
            },
            graphoptions: {
                maintainAspectRatio: true,
                title: {
                  display: true,
                  text: "Largest cities of Massachusetts"
                },
                legend: { display: true, position: "bottom" }
              }
        }
    }

    render() {
        return (
        <Grid container spacing={1}>
            <Grid item xs={12} align="center">
                <p>The default page</p> 
            </Grid>
            <Grid item xs={12} align="center">
                <Line options={this.state.graphoptions} data={this.state.datafirst} />   
            </Grid>
        </Grid>);
    }
}
