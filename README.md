To use application:

1. In this dir, run 'python3 app.py'

2. Open website link displayed in Terminal (i.e. type http://127.0.0.1:5001 in Safari)

Example:

![Start Terminal](./static/images/start.png)

*

To set Location Permissions:

1. In this dir, run 'openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes'

2. It will ask for information. Enter: US, Florida, Gainesville, -, -, 127.0.0.1, -

3. It is important to use the proper I.P. of the url being accessed ^

*


To connect to mobile, do the following:

1. Download Ngrok from ngrok.com

2. Run 'python3 app.py'

3. In a new terminal, run 'ngrok http http://localhost:5001'

4. On your mobile device, run the link it gives you.

    i.e. https://a2ec-184-185-222-10.ngrok-free.app

Completed Work:
* Manual Controller Interface: To be Connected to the Controller Node to Steer the Car.

  a. Drag lever up/down to accelerate/decelerate

  b. Press Left or Right Buttons to steer left/right

  c. Can also use up/down/left/right computer keys to accelerate/decelerate/steer left/steer right


* GPS: To be Connected with the GPS Locations of the:

  a. User (COMPLETED)

  b. R/C Car (to do)

  c. Drone (to do)


* Video Feed: To be Connected to Computer Vision

  a. Will show what the car is 'seeing'

  b. Currently connects to local computer camera to prove its function


* Website Interfacing: Using Flask

  a. Primary mode of Drone Home interraction

  b. Is Perfectly Formatted, thus far


* Mobile Interfacing: To work on phones as well as computers

  a. Works, but incomplete. Formatting needs work.


Current Design:

![Connection Portal](./static/images/currentweb.png)


BlueSky Design:

![Controller Page](./static/images/mainpage.png)

BUGS!:

Using mobile is INCOMPLETE, and currently shows up like this:

![Moile Page](./static/images/mobileincompleteee.png)

Thus, the mobile page is currently unusable and should not be run. Only run the website via local computer url, such as Safari, Chrome, or DuckDuckGo.


