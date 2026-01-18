// ChessMate CI/CD Pipeline
// Jenkinsfile for self-hosted Jenkins with hardware access
// Copyright (c) 2025 Vipin M - MIT License
//
// Tiered Testing Strategy:
//   - Unit Tests: Fail fast (no || true) - core logic must be correct
//   - Integration Tests: Mark unstable but continue - may have environmental issues
//   - Hardware Tests: Allow failure (|| true) - hardware may not always be available

pipeline {
    agent {
        label 'chessmate'  // Use agent with hardware access
    }

    parameters {
        booleanParam(name: 'RUN_HARDWARE_TESTS', defaultValue: false,
                     description: 'Run hardware tests (requires physical hardware)')
    }

    environment {
        ROS_DISTRO = 'jazzy'
        // Use dedicated CI workspace to avoid conflicts with development workspace
        CI_WORKSPACE = "${env.HOME}/ChessMate_CI/chessmate_ws"
        VENV_PATH = "${env.HOME}/ChessMate/chessmate_env"  // Reuse existing venv
        ROS_DOMAIN_ID = '42'  // Use different domain ID to isolate CI from dev
        PYTHONDONTWRITEBYTECODE = '1'  // Prevent .pyc file conflicts
    }

    options {
        timeout(time: 30, unit: 'MINUTES')
        buildDiscarder(logRotator(numToKeepStr: '10'))
        timestamps()
        disableConcurrentBuilds()  // Prevent hardware conflicts
    }

    stages {
        stage('Checkout') {
            steps {
                checkout scm
                sh '''
                    echo "=== Git Information ==="
                    git log -1 --oneline
                    echo "Branch: ${GIT_BRANCH:-unknown}"
                    echo "Commit: ${GIT_COMMIT:-unknown}"
                '''
            }
        }

        stage('Environment Setup') {
            steps {
                sh '''
                    echo "=== Environment Setup ==="
                    echo "ROS Distro: ${ROS_DISTRO}"
                    echo "CI Workspace: ${CI_WORKSPACE}"
                    echo "Virtual Env: ${VENV_PATH}"

                    # Create CI workspace directory structure
                    mkdir -p ${CI_WORKSPACE}/src

                    # Verify ROS2 installation
                    if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then
                        echo "✅ ROS2 ${ROS_DISTRO} found"
                    else
                        echo "❌ ROS2 ${ROS_DISTRO} not found"
                        exit 1
                    fi

                    # Verify Python virtual environment exists
                    if [ -f ${VENV_PATH}/bin/activate ]; then
                        echo "✅ Python venv found"
                    else
                        echo "❌ Python venv not found at ${VENV_PATH}"
                        echo "Please create it manually: python3 -m venv ${VENV_PATH}"
                        exit 1
                    fi
                '''
            }
        }

        stage('Sync Source') {
            steps {
                sh '''
                    echo "=== Syncing Source to CI Workspace ==="
                    # Copy source to CI workspace (preserving symlinks)
                    rsync -av --delete --exclude='build' --exclude='install' --exclude='log' \
                          --exclude='*.pyc' --exclude='__pycache__' --exclude='.git' \
                          ${WORKSPACE}/ ${CI_WORKSPACE}/src/chessmate/

                    echo "✅ Source synced to ${CI_WORKSPACE}/src/chessmate/"
                '''
            }
        }

        stage('Install Dependencies') {
            steps {
                sh '''
                    source ${VENV_PATH}/bin/activate
                    pip install --upgrade pip --quiet
                    pip install pytest pytest-cov --quiet

                    if [ -f ${CI_WORKSPACE}/src/chessmate/requirements.txt ]; then
                        pip install -r ${CI_WORKSPACE}/src/chessmate/requirements.txt --quiet
                        echo "✅ Dependencies installed"
                    else
                        echo "⚠️ requirements.txt not found, skipping"
                    fi
                '''
            }
        }

        stage('Build') {
            steps {
                sh '''
                    source /opt/ros/${ROS_DISTRO}/setup.bash
                    source ${VENV_PATH}/bin/activate
                    cd ${CI_WORKSPACE}

                    echo "=== Building ChessMate ==="
                    colcon build --symlink-install --packages-select chessmate 2>&1 | tail -20

                    if [ -f ${CI_WORKSPACE}/install/setup.bash ]; then
                        echo "✅ Build complete"
                    else
                        echo "❌ Build failed - install/setup.bash not found"
                        exit 1
                    fi
                '''
            }
        }

        stage('Unit Tests') {
            steps {
                sh '''
                    source /opt/ros/${ROS_DISTRO}/setup.bash
                    source ${VENV_PATH}/bin/activate
                    source ${CI_WORKSPACE}/install/setup.bash
                    cd ${CI_WORKSPACE}/src/chessmate

                    echo "=== Running Unit Tests (Fail Fast Mode) ==="
                    echo "Unit test failures will fail the entire pipeline."
                    echo ""

                    # Run pytest without || true - failures will fail the build
                    python3 -m pytest test/unit/ -v --tb=short \
                        --junitxml=unit-test-results.xml \
                        -o junit_family=xunit2
                '''
                // NO || true here - unit test failures MUST fail the pipeline
            }
            post {
                always {
                    junit testResults: '**/unit-test-results.xml', allowEmptyResults: true
                }
            }
        }

        stage('Integration Tests') {
            when {
                anyOf {
                    branch 'main'
                    branch 'develop'
                    changeRequest()  // Also run on PRs
                }
            }
            steps {
                script {
                    // Use returnStatus to capture exit code without failing immediately
                    def testResult = sh(
                        script: '''
                            source /opt/ros/${ROS_DISTRO}/setup.bash
                            source ${VENV_PATH}/bin/activate
                            source ${CI_WORKSPACE}/install/setup.bash
                            cd ${CI_WORKSPACE}/src/chessmate

                            echo "=== Running Integration Tests (Unstable Mode) ==="
                            echo "Integration test failures mark build UNSTABLE but continue."
                            echo ""

                            python3 -m pytest test/integration/ -v --tb=short \
                                --junitxml=integration-test-results.xml \
                                -o junit_family=xunit2
                        ''',
                        returnStatus: true
                    )

                    if (testResult != 0) {
                        // Mark build as UNSTABLE (yellow) but don't fail (red)
                        unstable("⚠️ Integration tests failed with exit code ${testResult}")
                    } else {
                        echo "✅ Integration tests passed"
                    }
                }
            }
            post {
                always {
                    junit testResults: '**/integration-test-results.xml', allowEmptyResults: true
                }
            }
        }

        stage('Hardware Tests') {
            when {
                expression {
                    return params.RUN_HARDWARE_TESTS == true
                }
            }
            steps {
                sh '''
                    source /opt/ros/${ROS_DISTRO}/setup.bash
                    source ${VENV_PATH}/bin/activate
                    source ${CI_WORKSPACE}/install/setup.bash
                    cd ${CI_WORKSPACE}/src/chessmate

                    echo "=== Running Hardware Tests (Allow Failure Mode) ==="
                    echo "Hardware test failures are tolerated - hardware may not be available."
                    echo ""

                    echo "Checking serial port access..."
                    ls -la /dev/ttyUSB* /dev/ttyACM* /dev/chessboard /dev/robot 2>/dev/null || echo "No serial devices found"
                    echo ""

                    # || true allows hardware tests to fail without failing the pipeline
                    python3 -m pytest test/hardware/ -v --tb=short \
                        --junitxml=hardware-test-results.xml \
                        -o junit_family=xunit2 \
                        -m "not requires_hardware or hardware" || true
                '''
            }
            post {
                always {
                    junit testResults: '**/hardware-test-results.xml', allowEmptyResults: true
                }
            }
        }
    }

    post {
        success {
            echo '''
╔══════════════════════════════════════════════════════════════╗
║  ✅ PIPELINE COMPLETED SUCCESSFULLY                          ║
║                                                              ║
║  All unit tests passed. Build is ready for deployment.      ║
╚══════════════════════════════════════════════════════════════╝
'''
        }
        unstable {
            echo '''
╔══════════════════════════════════════════════════════════════╗
║  ⚠️  PIPELINE COMPLETED WITH WARNINGS                        ║
║                                                              ║
║  Some integration or hardware tests may have failed.         ║
║  Review test results before deploying.                       ║
╚══════════════════════════════════════════════════════════════╝
'''
        }
        failure {
            echo '''
╔══════════════════════════════════════════════════════════════╗
║  ❌ PIPELINE FAILED                                          ║
║                                                              ║
║  Unit tests or build failed. Fix issues before merging.     ║
╚══════════════════════════════════════════════════════════════╝
'''
        }
        always {
            // Archive test results for historical tracking
            archiveArtifacts artifacts: '**/*-test-results.xml', allowEmptyArchive: true

            // Clean up workspace but preserve CI workspace for debugging
            cleanWs(cleanWhenNotBuilt: false, deleteDirs: true, disableDeferredWipeout: true,
                    patterns: [[pattern: '*-test-results.xml', type: 'EXCLUDE']])
        }
    }
}

