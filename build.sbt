organization := "edu.berkeley.cs"
version := "3.0"
name := "boom"
scalaVersion := "2.12.10"
libraryDependencies ++= Seq(
  "org.scala-lang" % "scala-compiler" % scalaVersion.value,
  "com.lihaoyi" %% "os-lib" % "0.7.8"
)
