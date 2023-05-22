## Code Formatting
### Spotless
[Spotless](https://github.com/diffplug/spotless) is a tool that automatically formats code to follow a certain convention. It allows multiple developers to write code but still maintain some uniformity in bracketing, indentation, whitespace, etc of the code.

This implementation of spotless mainly enforces the [Google Java Style](https://google.github.io/styleguide/javaguide.html), replaced tab indentation with spaces, and trims any trailing whitespace in files.

#### Summary of [changes](https://github.com/BHSRobotix/RomiTutorial2023/commit/7846aa835e5258fc29c79e32fe031e1c236fa52d?diff=split):
1. `build.gradle`
   1. [Install the spotless plug-in](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/code-formatting.html#spotless)
   1. Configure the spotless settings for the various file types (java, gradle, xml, etc)
   1. Disable automatics spotless execution during a build
      * *By default, spotless is configured to run during a build.  This is generally undesirable because we don't want formatting errors to cause build failures and hinder developent, debugging, and testing.  We generally want to run and apply the spotless changes prior to committing and/or pushing the code to a source code repository (e.g. git)*

### Apply Spotless Code Formatting

Once spotless is configured, you can run and apply the spotless changes by running `spotlessApply` via Gradle.

From vscode, you can run spotless as follows:
1. Open the vscode command palette: `Ctrl-Shift-P`
1. Type/Select: `WPILib: Run a command in Gradle`
1. Type: `spotlessApply`

#### Summary of [changes](https://github.com/BHSRobotix/RomiTutorial2023/commit/98a7cd1f0fe42dc9d9b2fb1269e285abfadc575a?diff=split):
1. Code reformatted by spotless

To keep the code base as uniform as possible, ***all subsequent changes should be processed by spotless prior to pushing it to the shared repository.***
