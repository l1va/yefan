#!groovy
def projectName = 'yefan'

pipeline{
	agent none
	stages{
		stage('Test'){
			agent{
				docker{
					label 'agent-1'
					image "base-${projectName}:latest"
					registryUrl 'https://robolab.innopolis.university:5000'
					registryCredentialsId 'jenkins-service'
					args '-u 1002:1002'
				}
			}
			steps{
					sh "mkdir -p catkin_ws/src/${projectName}"
					sh "cp -r `ls | grep -v catkin_ws`  catkin_ws/src/${projectName}"
					dir("catkin_ws"){
						sh "/bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make;'"
						sh "uname -a"
						sh "./src/${projectName}/run_all_tests.sh"
						echo "TEST SUCCESS!"
						deleteDir()
					}	
			}
		}
		stage('build'){
			agent{
				label 'agent-1'
			}
			environment{
				registry = "robolab.innopolis.university:5000/robo-${projectName}"
				tag_beta ='latest'
			}
			steps{
					script{
						def dockerfile = "Dockerfile-${projectName}"
						def image = docker.build("${env.registry}:${env.tag_beta}", "-f ${dockerfile} ./")
					
						withDockerRegistry([credentialsId: "jenkins-service", url: "https://robolab.innopolis.university:5000"]){
							image.push()
							}
						}
					deleteDir()	
			}
		}
	}
}
