within VDCWorkbenchModels.VehicleComponents.Controllers.VDControl.GeoPFC;
block TimeIndependentPathInterpolation "Time independent path interpolation"
  parameter Real e_long_gain=80 "TIPI Controller gain to force e_long to 0";
  parameter Real s_start=0 "Arc length value at start position";
  parameter String FilePath = ModelicaServices.ExternalReferences.loadResource(
    "modelica://VDCWorkbenchModels/Resources/Maps/Techlab2SBahn-NonOpt_TIPI.mat")
    "File where path table information is stored in table 'path_TIPI'";
  parameter Real maxArcLength = 2.312560625428274e+03 "Maximum arc length value on path file";
  //Can be improved in final version (store in MAT file)
  Modelica.Blocks.Tables.CombiTable1Ds Path(
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments,
    columns={2,3,4,5,6},
    tableOnFile=true,
    fileName=FilePath,
    tableName="path_TIPI",
    table=[0,0.846749729869016,4.10764354034983,0.433956908644525,13.3059227591937,0.0252120852893905,-0.414642942073828,6.82957089558341,2.10814240181186,1.38571618511625;
3.65235398017224,4.08775672912079,5.78899878915736,0.522124956194843,12.4552581006634,0.0235328839332705,2.59158745197394,8.38928290711868,5.58392600626763,3.18871467119604;
7.30470796034448,7.17055071659482,7.74541529641686,0.609854249113591,11.5420679254073,0.0252248416014213,5.45230674782195,10.2046098116547,8.88879468536769,5.28622078117906;
10.9570619405167,10.0551982987855,9.98290415152898,0.715051522226309,10.5501282817897,0.0345062695069890,8.08822933905007,12.2480825355503,12.0221672585209,7.71772576750766;
14.6094159206890,12.6378682054120,12.5612722526811,0.849438434556127,9.45468602525954,0.0270128755863562,10.3851392164983,14.5424870574179,14.8905971943256,10.5800574479444;
18.2617699008612,15.0186663667636,15.3306277976317,0.823505611841508,8.21444866896493,-0.0506198336171550,12.8180675619577,17.3695895187748,17.2192651715695,13.2916660764886;
21.9141238810334,17.8242044154010,17.6482362156917,0.532193849483806,6.75110709461249,-0.0989125098975898,16.3019294394808,20.2333240087244,19.3464793913213,15.0631484226589;
25.5664778612057,21.2714941541974,18.7227557385780,0.00531134539143110,5.03894515609738,-0.206097592833567,21.2555601929405,21.7227134230926,21.2874281154542,15.7227980540633;
29.2188318413779,24.5504050030210,17.3413547732766,-0.757882196717818,5.24425196180167,-0.154822767843170,26.6125595267363,19.5202349260892,22.4882504793058,15.1624746204640;
32.8711858215502,26.4610249363649,14.2731736319388,-1.26650677767772,5.71040392919729,-0.148117813224477,29.3232051025008,15.1720199461538,23.5988447702289,13.3743273177237;
36.5235398017224,26.3931643487839,10.6964708497555,-1.95621059881356,5.09748440061999,-0.188597503270591,29.1730926573041,9.56864169625404,23.6132360402636,11.8243000032569;
40.1758937818946,24.2319260770953,7.78696005208761,-2.38873356532941,6.18659077584562,-0.0670198776781423,26.2831098798193,5.59774900528499,22.1807422743712,9.97617109889023;
43.8282477620669,21.3585336816428,5.53782990671722,-2.53803517926809,7.16981390068696,-0.0194037776770299,23.0612586951767,3.06786482026640,19.6558086681089,8.00779499316804;
47.4806017422391,18.3098529651755,3.52686165168623,-2.57303510097140,8.03404612067927,-0.00545827236794830,19.9251042348877,0.998826182059083,16.6946016954632,6.05489712131337;
51.1329557224113,15.2262419627242,1.56956759446726,-2.57225971396215,8.81405970092578,0.00515794991947629,16.8434529525374,-0.957214670475143,13.6090309729110,4.09634985940966;
54.7853097025836,12.1719823850064,-0.433093271775448,-2.55123337960746,9.53046049735675,0.00420170701799370,13.8419609521052,-2.92531549008391,10.5020038179075,2.05912894653301;
58.4376636827558,9.15026678196397,-2.48462773594126,-2.54031710919166,10.1966501138237,0.00209574885859942,10.8473510799320,-4.95847188737652,7.45318248399596,-0.0107835845060031;
62.0900176629281,6.14496848170819,-4.56016051496850,-2.53466018370966,10.8219023592457,0.00109317082775912,7.85601990309645,-7.02436485579846,4.43391706031992,-2.09595617413854;
65.7423716431003,3.14822087486178,-6.64802655370326,-2.53183138078151,11.4126051431748,0.000527120790977009,4.86623618938815,-9.10738081428729,1.43020556033541,-4.18867229311922;
69.3947256232725,0.155534246458154,-8.74171107614935,-2.53070241386581,11.9683805732090,9.35893428604314e-05,1.87632499512680,-11.1991241873898,-1.56525650221050,-6.28429796490891;
73.0470796034448,-2.83642944415159,-10.8364288659871,-2.53113389999546,12.4298749994680,-0.000328552256613806,-1.11669919531099,-13.2945842457836,-4.55615969299219,-8.37827348619054;
76.6994335836170,-5.83108214842875,-12.9272989747382,-2.53398888116645,12.3978049240935,-0.00242389064211397,-4.11837688606392,-15.3903541272404,-7.54378741079358,-10.4642438222359;
80.3517875637893,-8.84683296369748,-14.9875078767355,-2.55450036214245,11.6557361414483,-0.00894480674755246,-7.18500534055427,-17.4851725762014,-10.5086605868407,-12.4898431772695;
84.0041415439615,-11.9288869427534,-16.9466457697789,-2.60105385251476,10.6900177498497,-0.0168954464011914,-10.3850927886680,-19.5189403893378,-13.4726810968388,-14.3743511502200;
87.6564955241337,-15.1268773162818,-18.7088526566253,-2.68155405021761,9.61149290133796,-0.0285366007267763,-13.7949292244335,-21.3969587335169,-16.4588254081302,-16.0207465797337;
91.3088495043060,-18.4988658461401,-20.1027824651926,-2.83585854629323,8.39655123042107,-0.0585821066032765,-17.5958858854260,-22.9636612096606,-19.4018458068542,-17.2419037207246;
94.9612034844782,-22.0838418482779,-20.7290953475629,-3.12874725951042,7.07537364182596,-0.102648433332201,-22.0453067258027,-23.7288478447426,-22.1223769707531,-17.7293428503832;
98.6135574646504,-25.6428049603274,-20.0375288700971,-3.54228691506666,6.80633320181815,-0.107160185253414,-26.8129780770483,-22.7999001117849,-24.4726318436065,-17.2751576284093;
102.265911444823,-28.6795003430020,-18.0418494565919,-3.89389117970834,7.34331081018575,-0.0907332805911381,-30.7294566346853,-20.2322099753623,-26.6295440513187,-15.8514889378215;
105.918265424995,-30.8812336049335,-15.1497282005905,-4.23445886862835,7.12270711705007,-0.0985875219465344,-33.5450801646744,-16.5295548223646,-28.2173870451926,-13.7699015788164;
109.570619405167,-31.9174364196565,-11.6707245310274,-4.61413466382770,7.02556968580459,-0.101275697913907,-34.9029671995742,-11.9650134403733,-28.9319056397388,-11.3764356216814;
113.222973385339,-31.6272856581938,-8.04873561650656,-4.96659259794586,7.22382534442339,-0.0947503918905111,-34.5308772760627,-7.29431152049571,-28.7236940403250,-8.80315971251741;
116.875327365512,-30.1117756036212,-4.74685471061464,-5.32328178733817,7.01968480812461,-0.101846244310608,-32.5691842971888,-3.02605765320822,-27.6543669100537,-6.46765176802105;
120.527681345684,-27.5043526728462,-2.22142925077201,-5.70470753158697,7.07462094336984,-0.0973286110404657,-29.1446027283182,0.290458436909185,-25.8641026173743,-4.73331693845321;
124.180035325856,-24.1987483006995,-0.692640264811219,-5.96600012167895,7.87078220165560,-0.0495632630244646,-25.1344284897295,2.15771104403772,-23.2630681116696,-3.54299157366016;
127.832389306028,-20.6580486811392,0.193360442190785,-6.09418894804974,8.66380423955699,-0.0234161739354857,-21.2216683424551,3.13994030321240,-20.0944290198233,-2.75321941883083;
131.484743286201,-17.0507170129453,0.761969558802656,-6.15053345529222,9.39089310582465,-0.00857340814356142,-17.4475064899546,3.73561347005611,-16.6539275359361,-2.21167435245080;
135.137097266373,-13.4263335850207,1.21293300374838,-6.16188961751146,10.0527956306356,0.00161555281806474,-13.7893290200220,4.19089108172197,-13.0633381500193,-1.76502507422522;
138.789451246545,-9.80341247956010,1.67543130443238,-6.14769413704260,10.4602866382442,0.00632055490412372,-10.2086434672467,4.64793661924992,-9.39818149187350,-1.29707401038516;
142.441805226717,-6.19265786007913,2.22388688264913,-6.11292011575957,9.86756522245305,0.0134208596442860,-6.70098899529557,5.18050648228814,-5.68432672486269,-0.732732716989888;
146.094159206890,-2.61549803880688,2.95718950707975,-6.03958878804881,8.72442242775189,0.0293656312921322,-3.33908159031924,5.86861988474307,-1.89191448729452,0.0457591294164352;
149.746513187062,0.845408269454243,4.10750510596319,-5.84922839853506,7.36354978700530,0.0918326005587445,-0.415984402488600,6.82943246119678,2.10680094139709,1.38557775072961])
    annotation (Placement(transformation(extent={{30,30},{50,50}})));

protected
  Modelica.Blocks.Interfaces.RealOutput sI_C[3] "Vehicle Position in inertial frame of reference"
    annotation (Placement(
      transformation(extent={{-10,-70},{-50,-30}})));
  Modelica.Blocks.Interfaces.RealOutput vI_C[2] "Vehicle speed in inertial frame I"
    annotation (Placement(
      transformation(extent={{-10,-110},{-50,-70}})));
public
  Real sDot "Time derivative of arc length";
  Real tvI_P[2] "Tangent of desired path in inertial frame I (normalized)";
  Real nvI_P[2]
    "Vector normal to tvI_P; rotated tvI_P +90° in inertial frame I (normalized)";
  //Real vI_C[2] "Vehicle speed in inertial frame of reference"; // -> Not considered since this value is a direct input
  Real e[2] "Distance vector from path reference to vehicle position in inertial frame I";
  Real e_lat "Distance of vehicle to path in direction of nvI_P";
  Real e_long "Distance of vehicle to path in direction of nvI_P";
  Real kappa
    "Curvature of path, derivative of psi_ref with respect to parameter s";
  Real lambda[5];
  Modelica.Blocks.Sources.RealExpression realExpression_sDot(y=sDot)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  Modelica.Blocks.Routing.RealPassThrough sampler
    annotation (Placement(transformation(extent={{-10,30},{10,50}})));
  Modelica.Blocks.Continuous.Integrator sIntegrator(y_start=s_start)
    annotation (Placement(transformation(extent={{-66,30},{-46,50}})));
protected
  VDCWorkbenchModels.Utilities.Interfaces.MotionDemandBus motionDemandBus
    annotation (Placement(transformation(extent={{70,-10},{90,10}})));
public
  Modelica.Blocks.Sources.RealExpression realExpression_e_long(y=e_long)
    annotation (Placement(transformation(extent={{40,-50},{60,-30}})));
  Modelica.Blocks.Sources.RealExpression realExpression_e_lat(y=e_lat)
    annotation (Placement(transformation(extent={{40,-72},{60,-52}})));
  Utilities.Blocks.Modulo modulo(k=maxArcLength) annotation (Placement(transformation(extent={{-38,30},{-18,50}})));
  VDCWorkbenchModels.Utilities.Interfaces.ControlBus controlBus annotation (
      Placement(transformation(extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={100,0})));
  Modelica.Blocks.Interfaces.RealInput v_scl "Down scale vector for velocity "
    annotation (Placement(transformation(extent={{-140,40},{-100,80}}),
        iconTransformation(extent={{-140,40},{-100,80}})));
protected
  VehicleInterfaces.Interfaces.ChassisBus chassisBus
    annotation (Placement(transformation(extent={{10,-90},{30,-70}})));
initial equation
//kappa = Bahn.y[5];
equation
  tvI_P = {cos(lambda[3]),sin(lambda[3])};
  nvI_P = [0,-1; 1,0]*tvI_P;  //Rotate tvI_P around 90°
  //vI_C = [cos(sI_C[3]),-sin(sI_C[3]); sin(sI_C[3]),cos(sI_C[3])]*vC_C; // -> Not considered since this value is a direct input
  e = sI_C[1:2] - lambda[1:2]; // Calculate distance error in inertial frame I
  e_long = e*tvI_P;  // Calculate distance error in direction of path tangent
  e_lat = e*nvI_P;  // Calculate distance error in direction normal to path tangent

  // Calculate reference velocity
  //sDot=(cos(lambda[3]-sI_C[3])*vC_C[1]+sin(lambda[3]-sI_C[3])*vC_C[2])/(1-e_lat*kappa)+e_long_gain*e_long;
  sDot=(cos(lambda[3])*vI_C[1]+sin(lambda[3])*vI_C[2])/(1-e_lat*kappa)+e_long_gain*e_long;
  //lambda[:] = Path.y[:];
  lambda[1] = Path.y[1] "x-position";
  lambda[2] = Path.y[2] "y-position";
  lambda[3] = Path.y[3] "psi orientation";
  lambda[4] = Path.y[4]*v_scl "scaled long speed";
  lambda[5] = Path.y[5] "curvature";
  kappa =Path.y[5];
  if sIntegrator.y > (maxArcLength-1.0) then
    terminate("Vehicle reached end of path");
  end if;

connect(Path.u, sampler.y) annotation (Line(
    points={{28,40},{11,40}},
    color={0,0,127},
    smooth=Smooth.None));
  connect(modulo.y, sampler.u) annotation (Line(points={{-17,40},{-12,40}},
        color={0,0,127}));
  connect(realExpression_sDot.y, sIntegrator.u) annotation (Line(points={{-79,40},{-68,40}},
        color={0,0,127}));
  connect(sIntegrator.y, modulo.u) annotation (Line(points={{-45,40},{-40,40}},
        color={0,0,127}));
  connect(realExpression_sDot.y, motionDemandBus.s_dot) annotation (Line(points={{-79,40},{-74,40},{-74,-2},{78,-2},{78,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(sampler.y, motionDemandBus.arc_length) annotation (Line(points={{11,40},{20,40},{20,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(realExpression_e_long.y, motionDemandBus.e_long) annotation (Line(
        points={{61,-40},{80,-40},{80,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(realExpression_e_lat.y, motionDemandBus.e_lat) annotation (Line(
        points={{61,-62},{82,-62},{82,0},{80,0}}, color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(motionDemandBus, controlBus.motionDemandBus) annotation (Line(
        points={{80,0},{80,-0.1},{100.1,-0.1}},
        color={255,204,51},
        thickness=0.5));
  connect(Path.y[2], motionDemandBus.y_path) annotation (Line(points={{51,40},{80,40},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(Path.y[1], motionDemandBus.x_path) annotation (Line(points={{51,40},{54,40},{54,42},{82,42},{82,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(Path.y[3], motionDemandBus.psi_path) annotation (Line(points={{51,40},{51,38},{54,38},{54,34},{74,34},{74,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(Path.y[4], motionDemandBus.v_path) annotation (Line(points={{51,40},{54,40},{54,36},{76,36},{76,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(Path.y[5], motionDemandBus.kappa_path) annotation (Line(points={{51,40},{51,38},{78,38},{78,0},{80,0}},
        color={0,0,127}),
      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
  connect(chassisBus, controlBus.chassisBus) annotation (Line(
      points={{20,-80},{100.1,-80},{100.1,-0.1}},
      color={255,204,51},
      thickness=0.5));
  connect(sI_C[1], chassisBus.position_x) annotation (Line(points={{-30,-56.6667},{18,-56.6667},{18,-80},{20,-80}},
        color={0,0,127}));
  connect(sI_C[2], chassisBus.position_y) annotation (Line(points={{-30,-50},{20,-50},{20,-80}},
        color={0,0,127}));
  connect(sI_C[3], chassisBus.yawAngle) annotation (Line(points={{-30,-43.3333},{22,-43.3333},{22,-80},{20,-80}},
        color={0,0,127}));
  connect(vI_C[1], chassisBus.velocity_dx) annotation (Line(points={{-30,-95},{-30,-94},{20,-94},{20,-80}},
        color={0,0,127}));
  connect(vI_C[2], chassisBus.velocity_dy) annotation (Line(points={{-30,-85},{-30,-84},{18,-84},{18,-80},{20,-80}},
        color={0,0,127}));
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
      graphics={
        Polygon(
          points={{0,100},{100,-100},{-100,-100},{0,100}},
          lineColor={28,108,200},
          fillColor={255,255,0},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-212,108},{202,78}},
          textColor={0,0,255},
          textString=""),
        Line(
          points={{-92,-86},{-68,-20},{-24,30},{64,98}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{-74,-96},{-46,-28},{-2,16},{96,90}},
          color={28,108,200},
          pattern=LinePattern.Dash,
          thickness=0.5),
        Line(
          points={{-38,-98},{0,-28},{98,54}},
          color={238,46,47},
          thickness=0.5),
        Line(
          points={{-14,30},{-14,4},{10,4}},
          color={238,46,47},
          thickness=0.5),
        Text(
          extent={{-28,2},{0,-12}},
          textColor={238,46,47},
          textString="s"),
        Text(
          extent={{-10,-50},{70,-80}},
          textColor={0,0,0},
          textString="TIPI")}),
    Diagram(coordinateSystem(preserveAspectRatio=false,
        extent={{-100,-100},{100,100}}), graphics={
        Text(
          extent={{0,100},{80,50}},
          textColor={28,108,200},
          textString="Input arc length
'X-Position        '
'Y-Position        '
'Psi-Orientation   '
'longitudinal Speed'
'Curvature         '
")}));
end TimeIndependentPathInterpolation;
