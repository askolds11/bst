var chart;

async function onClick() {
    var datums = document.getElementById("datums").value;
    var sensor_num = document.getElementById("sensor_num").value;

    if (!datums || !sensor_num) {
        return;
    }

    var rawData = await fetch(`/${datums}/${sensor_num}.txt`)
        .then((response) => response.text())

    var data = rawData.split('\n'); // split by lines
    data = data.filter(x => x);
    data = data.map(x => x.split(','));

    var mappedData = data.map(x => ({date: Number(x[0]), temp: Number(x[3]), humidity: Number(x[4])}));

    if (chart != undefined) {
        chart.destroy();
    }

    (async function() {     
        chart = new Chart(
            document.getElementById('graph'),
            {
                type: 'line',
                data: {
                    labels: mappedData.map(row => row.date),
                    datasets: [
                        {
                            label: 'Temp',
                            data: mappedData.map(row => row.temp),
                            borderColor: '#fda04c',
                            backgroundColor: '#fda04c',
                            yAxisID: 'y',
                        },
                        {
                            label: 'Humidity',
                            data: mappedData.map(row => row.humidity),
                            borderColor: '#3ea1e7',
                            backgroundColor: '#3ea1e7',
                            yAxisID: 'y1',
                        }
                    ]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    legend: {
                        position: 'bottom',
                    },
                    layout: {
                        padding: {
                            top: 35,
                            right: 15,
                            left:20
                        }
                    },
                    scales: {
                        x: {
                            type: 'time',
                            time: {
                                unit: 'hour',
                                displayFormats: {
                                    hour: 'HH:mm'
                                },
                                tooltipFormat: 'D MMM YYYY - HH:mm:ss.mmm'
                            }
                        },
                        y: {
                            type: 'linear',
                            title: {
                                display: true,
                                text: 'Temperature °C'
                            },
                            position: 'left',
                            suggestedMin: 0,
                            suggestedMax: 40
                          },
                          y1: {
                            type: 'linear',
                            title: {
                                display: true,
                                text: 'Humidity %'
                            },
                            position: 'right',
                            min: 0,
                            max: 100,
                            
                            // grid line settings
                            grid: {
                              drawOnChartArea: false, // only want the grid lines for one axis to show up
                            },
                          },
                    }
                }
            }
        );
    })();
}

document.getElementById("load").addEventListener("click", onClick);