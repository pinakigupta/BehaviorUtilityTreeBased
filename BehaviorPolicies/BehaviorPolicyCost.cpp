#include "../include/BehaviorOptimalPolicy.hpp"


const int COUNT=5;
const double TEXT_LENGTH = 25;

int MaxPolicyCostTreeHight(const BehaviorTertiary::BehaviorPolicyCostTree *root) {
	if (!root) return 0;
	int MaxChildHight = 0;
	int ChildHight = 0;
	if(root->ChildBehaviorPolicyCosts.size()==0)
		return 1;
	for (int i = 0; i < root->ChildBehaviorPolicyCosts.size(); i++){
		ChildHight = MaxPolicyCostTreeHight(root->ChildBehaviorPolicyCosts[i]);
		if(ChildHight>MaxChildHight){
			MaxChildHight = ChildHight;
		}
	}
	return MaxChildHight+1 ;
}


void Policy::PrintBehaviorPolicyCostTree(std::ostream& os,const BehaviorTertiary::BehaviorPolicyCostTree *root,int indent)
{
	// Base case
	if (root == NULL)
		return ;
	std::string indentstr(indent, ' ');
	lcmprint::Printbehaviorpolicycosts(os,root->RootBehaviorPolicyCost,indentstr);
	if(root->ChildBehaviorPolicyCosts.size()==0)
		return;
	// level++;
	//  os<<" Following Level is "<<level<<endl;
	for (int i = 0; i < root->ChildBehaviorPolicyCosts.size(); i++)
		PrintBehaviorPolicyCostTree(os,root->ChildBehaviorPolicyCosts[i],indent-15+30*i);



	// Store marker at the end of children

}

// Print the arm branches (eg, /    \ ) on a line
void printBranches(int branchLen, int nodeSpaceLen, int startLen, int nodesInThisLevel, const deque<BehaviorTertiary::BehaviorPolicyCostTree*>& nodesQueue, ostream& os) {
	deque<BehaviorTertiary::BehaviorPolicyCostTree*>::const_iterator iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel / 2; i++) {
		os << ((i == 0) ? setw(startLen-1) : setw(nodeSpaceLen-2)) << "" << ((*iter++) ? "/" : " ");
		os << setw(branchLen+TEXT_LENGTH) << "" << ((*iter++) ? "\\" : " ");
	}
	os << endl;
	//os<<" Above branchLen "<<branchLen<<" nodeSpaceLen "<<nodeSpaceLen<<" startLen "<<startLen<<" nodesInThisLevel "<<nodesInThisLevel<<" nodesQueue size "<<nodesQueue.size()<<endl;
}

// Print the branches and node (eg, ___10___ )
void printNodes(int branchLen, int nodeSpaceLen, int startLen, int nodesInThisLevel, const deque<BehaviorTertiary::BehaviorPolicyCostTree*>& nodesQueue, ostream& os) {
	deque<BehaviorTertiary::BehaviorPolicyCostTree*>::const_iterator iter = nodesQueue.begin();
	std::string indent;
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(startLen) : setw(nodeSpaceLen)) << "" << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>0)) ? setfill('_') : setfill(' '));
		os << setw(branchLen+2) ;
		if(*iter){
			os<<" BehaviorType =  "<<(*iter)->RootBehaviorPolicyCost.BehaviorType;
		}

		os << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>1)) ? setfill('_') : setfill(' ')) << setw(branchLen-TEXT_LENGTH) << "" << setfill(' ');
		if(i==nodesInThisLevel-1)
			os<<endl;
	}

	iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(startLen) : setw(nodeSpaceLen)) << "" << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>0)) ? setfill(' ') : setfill(' '));
		os << setw(branchLen+2) ;
		if(*iter){
			os<<(*iter)->RootBehaviorPolicyCost.SpatialTask;
		}
		if(i==nodesInThisLevel-1)
			os<<endl;
	}

	iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(startLen) : setw(nodeSpaceLen)) << "" << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>0)) ? setfill(' ') : setfill(' '));
		os << setw(branchLen+2) ;
		if(*iter){
			os<<" TemporalRule = "<<(*iter)->RootBehaviorPolicyCost.TemporalRule;
		}
		if(i==nodesInThisLevel-1)
			os<<endl;
	}

	iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(startLen) : setw(nodeSpaceLen+10)) << "" << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>0)) ? setfill(' ') : setfill(' '));
		os << setw(branchLen+2) ;
		if(*iter){
			os<<" Behavior Policy cost = "<<(*iter)->RootBehaviorPolicyCost.PolicyCost;
		}
		if(i==nodesInThisLevel-1)
			os<<endl;
	}

	iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(startLen) : setw(nodeSpaceLen+10)) << "" << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>0)) ? setfill(' ') : setfill(' '));
		os << setw(branchLen+2) ;
		if(*iter){
			os<<" Temporal Maneuver cost = "<<(*iter)->RootBehaviorPolicyCost.TemporalManeuverCost;
		}
		if(i==nodesInThisLevel-1)
			os<<endl;
	}

	iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(startLen) : setw(nodeSpaceLen+10)) << "" << ((*iter && ((*iter)->ChildBehaviorPolicyCosts.size()>0)) ? setfill(' ') : setfill(' '));
		os << setw(branchLen+2) ;
		if(*iter){
			os<<" SpatialManeuverCost = "<<(*iter)->RootBehaviorPolicyCost.SpatialManeuverCost;
		}
		if(i==nodesInThisLevel-1)
			os<<endl;
	}

	os << endl;
	//	os<<" Above branchLen "<<branchLen<<" nodeSpaceLen "<<nodeSpaceLen<<" startLen "<<startLen<<" nodesInThisLevel "<<nodesInThisLevel<<" nodesQueue size "<<nodesQueue.size()<<endl;

}

// Print the leaves only (just for the bottom row)
void printLeaves(int indentSpace, int level, int nodesInThisLevel, const deque<BehaviorTertiary::BehaviorPolicyCostTree*>& nodesQueue, ostream& os) {
	deque<BehaviorTertiary::BehaviorPolicyCostTree*>::const_iterator iter = nodesQueue.begin();
	for (int i = 0; i < nodesInThisLevel; i++, iter++) {
		os << ((i == 0) ? setw(indentSpace+2) : setw(2*level+2));
		if(*iter){
			os<<" BehaviorType =  "<<(*iter)->RootBehaviorPolicyCost.BehaviorType;
		}
		else
			os<<" ";
	}
	os << endl;
}

// Pretty formatting of a binary tree to the output stream
// @ param
// level  Control how wide you want the tree to sparse (eg, level 1 has the minimum space between nodes, while level 2 has a larger space between nodes)
// indentSpace  Change this to add some indent space to the left (eg, indentSpace of 0 means the lowest level of the left node will stick to the left margin)
void PrintPrettyPolicyCostTree(const BehaviorTertiary::BehaviorPolicyCostTree *root, int indentSpace, ostream& os, int level = -1) {
	int h = MaxPolicyCostTreeHight(root);
	int nodesInThisLevel = 1;
	if(level<0)
		level=h;

	int branchLen = TEXT_LENGTH*((int)pow(2.0,h)-1) - (1+TEXT_LENGTH-level)*(int)pow(2.0,h-1);  // eq of the length of branch for each node of each level
	int nodeSpaceLen = TEXT_LENGTH + (level+1)*(int)pow(2.0,h);  // distance between left neighbor node's right arm and right neighbor node's left arm
	int startLen = branchLen + (1+TEXT_LENGTH-level) + indentSpace;  // starting space to the first node to print of each level (for the left most node of each level only)

	deque<BehaviorTertiary::BehaviorPolicyCostTree*> nodesQueue;
	//nodesQueue.push_back(root);
	for (int r = 0; r < h; r++) {
		printBranches(branchLen, nodeSpaceLen, startLen, nodesInThisLevel, nodesQueue, os);
		branchLen = branchLen/2 - 1;
		nodeSpaceLen = nodeSpaceLen/2 + 1;
		startLen = branchLen + (3-level) + indentSpace;
		printNodes(branchLen, nodeSpaceLen, startLen, nodesInThisLevel, nodesQueue, os);

		for (int i = 0; i < nodesInThisLevel; i++) {
			BehaviorTertiary::BehaviorPolicyCostTree *currNode = nodesQueue.front();
			nodesQueue.pop_front();
			if (currNode) {
				if(currNode->ChildBehaviorPolicyCosts.size()>1){
					nodesQueue.push_back(currNode->ChildBehaviorPolicyCosts[0]);
					nodesQueue.push_back(currNode->ChildBehaviorPolicyCosts[1]);
				}
				else if(currNode->ChildBehaviorPolicyCosts.size()>0){
					nodesQueue.push_back(currNode->ChildBehaviorPolicyCosts[0]);
					nodesQueue.push_back(NULL);
				}
				else {
					nodesQueue.push_back(NULL);
					nodesQueue.push_back(NULL);
				}


			} else {
				nodesQueue.push_back(NULL);
				nodesQueue.push_back(NULL);
			}
		}
		nodesInThisLevel *= 2;
	}
	//printBranches(branchLen, nodeSpaceLen, startLen, nodesInThisLevel, nodesQueue, os);
	//printLeaves(indentSpace, level, nodesInThisLevel, nodesQueue, os);
}


std::ostream& Policy::operator<<(std::ostream& os, const BehaviorTertiary::BehaviorPolicyCostTree *root){
	os<<" Maximum Depth of the Tree = "<<MaxPolicyCostTreeHight(root)<<endl;
	//Policy::PrintBehaviorPolicyCostTree(os,root);
	PrintPrettyPolicyCostTree(root,20,os);
	return os;
}

