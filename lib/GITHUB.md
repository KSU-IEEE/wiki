# GitHub Information  
This page holds the information on how to operate on Github. In the past we've been crushed for time, so we've branched when we've needed to without any real reguard for how to do it right. This page hopes to create a solid understanding for everybody to operate with adding code.  
# Issues and Branching 
## Creating an Issue  
We are going to start with how to open an issue. We will be using issues to better track who is in charge of a task. When opening an issue, follow these steps:   
![](/images/github/1_2.png)  
In this picture, we see the issue screen. To open any new issue, hit the new issue button--number 1.  

![](/images/github/3_4_5_6.png)  
In this picture, you see the screen you will see when you make a new issue. Follow the numbers, by first adding a title, then the description. This usually helps you organize your thoughts betters  
Then Assign this to yourself and anybody else that will be helping you on it.  
Finally submit the issue.  

## Branching  
Now that we have created our issue, we will need a branch to work on the issue. From the terminal, run the command:  
```
$ git branch <name of branch>
$ git checkout <name of branch>
```  
If you are using the GUI, follow the instructions [here](https://www.attosol.com/create-and-merge-branches-using-github-desktop-client/)  

**NAME YOUR BRANCH CORRECTLY**.  The naming system we will be following is the following: `<the number of your issue>-<the name of your issue>`. For the name of your issue, use '-' for spaces. For example. I opened issue number 2 title "Your Issue Name". So when I branch, I will run `git branch 2-Your-Issue-Name` if I wanted to work on that branch.  

## Commiting code  
1.) **commit often**. This helps in improving the readability of your code.  
2.) **have indepth commit message**. This also helps in readability. The message `squashed a bug` doens't tell me alot about what you did. Rather, if you commit `add class outline--not compiled yet`. I know what you did, and that your code doesn't work

## Commenting on your issue  
To keep better information on our code, we will be adding comments on our issues as we write code. Because the branches correspond to the issue, we will have an easier time following the code.   
![](/images/github/7_8.png)  
Above shows a comment box, you will want to add in depth descriptions a long the way of writing, be sure to hit comment at the end!  

When you've commented, you'll see this:  
![](/images/github/comments.png)  
This shows a clear timeline of the work you will be doing. Comment often to ensure readability.  

## Merging into master  
Once you finish your feature, you will merge your branch into master. To do this hit the `open merge request` box on the main page of the the repository !()[imagers/github/pull]  
Then you will see this screen:
![](/images/github/9_10_11_12.png)  
This will alert me that you are ready to merge into master. When I approve your code, I will accept the merge request, close out your issue, and delete your branch. You can still see the old issues, however this keeps the issues clean.  
