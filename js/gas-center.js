var gasCenterClient = new ROSLIB.Service({
    ros : ros,
    name : '/gas_center',
    serviceType : 'eva_a/GasCenter'
});

function callGasCenter() {

    console.log("Trying to call GasCenter.");
    var request = new ROSLIB.ServiceRequest({
    });
    
    gasCenterClient.callService(request, function(result) {
        console.log('Result for service call on '
            + gasCenterClient.name
            + ': '
            + result.success);
    });
}