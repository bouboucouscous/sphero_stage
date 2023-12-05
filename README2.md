How use git 

**To Retrieve the Project:**

1. Clone the project:

   ```bash
   git clone https://github.com/bouboucouscous/ProjetWeb.git
   ```

2. Initialize a Git repository:

   ```bash
   git init
   ```

3. Fetch the latest updates from the remote repository:

   ```bash
   git fetch
   ```

4. Set your global Git username:

   ```bash
   git config --global user.name "Your Name"
   ```

5. Set your global Git email:

   ```bash
   git config --global user.eamil "Your Email"
   ```

**Workflow for Adding Modifications:**

1. Create a new branch for your work:

   ```bash
   git checkout -b yourBranchName
   ```

2. Add your changes to the staging area:

   ```bash
   git add .
   ```

3. Commit your changes with a descriptive message:

   ```bash
   git commit -m "Description of what you added"
   ```

4. Push your changes to the remote repository and set the upstream branch:

   ```bash
   git push --set-upstream origin yourBranchName
   ```

**If the branch already exists:**

1. Add your changes to the staging area:

   ```bash
   git add .
   ```

2. Commit your changes with a descriptive message:

   ```bash
   git commit -m "Description of what you added"
   ```

3. Push your changes to the remote repository:

   ```bash
   git push
   ```

**To Retrieve the Changes on the Main Branch:**

1. Switch to the main branch:

   ```bash
   git checkout main
   ```

2. Pull the latest changes from the remote repository:

   ```bash
   git pull
   ```

**To Retrieve the Latest Changes on Your Working Branch and Update It:**

1. Switch back to your working branch (replace `yourBranchName` with your branch's name):

   ```bash
   git checkout yourBranchName
   ```

2. Rebase your branch onto the main branch to include the latest changes:

   ```bash
   git rebase main
   ```

**When Pushing Changes After Rebasing:**

When pushing changes after rebasing, use the following command to force push:

```bash
git push --force
```

Please replace `yourBranchName` with the actual name of your branch when following these instructions.