var gasGradientClient = new ROSLIB.Service({
    ros : ros,
    name : '/eva/gas_gradient',
    serviceType : 'eva_a/GasGradient'
});

function callGasGradient() {

    console.log("Trying to call GasGradient.");
    var request = new ROSLIB.ServiceRequest({
    });
    
    gasGradientClient.callService(request, function(result) {
        console.log('Result for service call on '
            + gasGradientClient.name
            + ': '
            + result.success);
    });
}