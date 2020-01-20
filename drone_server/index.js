var express = require('express');
var https = require("https");
var fs = require("fs");
var app = express();
var bodyParser = require('body-parser');

app.set('views', __dirname + '/views');
app.set('view engine', 'ejs');
app.engine('html', require('ejs').renderFile);

app.use(bodyParser.json());
app.use(bodyParser.urlencoded({extended : true}));

// var server = app.listen(3000, function(){
//   console.log("Express server has started on port 3000")
// });

var gps_info;
// var isAlreadySaved = false;

app.get('/', function(req, res){
  res.render('index.html');
});

app.post('/', function(req, res){
  console.log("req.body :", req.body);
  gps_info = req.body;
  res.render('index.html');
});

app.get("/get", function (req, res) {
  console.log(gps_info);
  res.send(gps_info);
});

https.createServer({
  key: fs.readFileSync("./key.pem"),
  cert: fs.readFileSync("./cert.pem"),
  passphrase: "mansu"
}, app).listen(3000, function () {
  console.log("Express server has started on port 3000");
});