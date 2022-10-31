# Robotic project
For now use the repo to push code from the assignment and ideas for our project

## Git tutorial
At first clone this repository 
```BASH
git clone git@github.com:davidedema/robotic_project.git
```
Then add the remote repository (in order to push the changes in github)
```BASH
git remote add robotics git@github.com:davidedema/robotic_project.git
```
Now crete your own local branch
```BASH
git branch yourname_develop
```
To start working in your branch type
```BASH
git checkout yourname_develop
```
When you finish to modify something and you want to track in your local branch type
```
git add filenames
git commit -m "Type what you have changed"
git push origin yourname_develop
```
Type the last command only if you want to publish on github

Online you can find better git tutorials, this explain only the essential.