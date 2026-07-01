ThisBuild / scalaVersion     := "2.13.12"
ThisBuild / version          := "0.1.0"
ThisBuild / organization     := "com.fpga-crypto"

val chiselVersion = "3.6.0"

lazy val root = (project in file("."))
  .settings(
    name := "kalman-filter",
    libraryDependencies ++= Seq(
      "edu.berkeley.cs" %% "chisel3"       % chiselVersion,
      "edu.berkeley.cs" %% "chiseltest"    % "0.6.0" % Test,
    ),
    scalacOptions ++= Seq(
      "-language:reflectiveCalls",
      "-deprecation",
      "-feature",
      "-Xcheckinit",
    ),
    addCompilerPlugin("edu.berkeley.cs" % "chisel3-plugin" % chiselVersion cross CrossVersion.full),
  )
