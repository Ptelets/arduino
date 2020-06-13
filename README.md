# There is a final code for my project.

The initially my plan was:
   - Put together two projects with pollution monitoring “https://www.instructables.com/id/Get-Started-Building-a-PM-Monitoring-Station/“
and a small weather station “https://www.instructables.com/id/Remote-Control-With-NodeMCU-and-Web-UI/"  together.

   - Pollution Monitoring on Arduino will send data to the computer via the WiFi Module (if I find one) or via the cable.
   - Weather station with temperature on ESP866 with FiWi.
   - Those data then will be got from a computer and will be available on a small website.

After learning more about Arduino Nano 33 IOT with the ability to make my webserver, so easy.

I transformed my idea to put all sensors: CO2, PM2.5, DHT22 plus internals on one small, but very powerful Arduino Nano 33 IOT.

All test connectivity with each of the sensors was done on Arduino Uno and Arduino nano 33 IOT.

PM2.5 sensor is so shy, in 5-10 minutes goes down and returns back to on-line mode in the same timne range.
I guess, it can be crossover of logic inside on Arduino... Need additional investigation.

