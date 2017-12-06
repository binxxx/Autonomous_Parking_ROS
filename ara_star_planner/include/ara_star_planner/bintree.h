#include <stdio.h>
#include <cstddef>
#include <cmath>

template <class KeyClass,class ValueClass>
class BinaryTree
{
protected:
	class Node
	{
	public:
		KeyClass key;
		ValueClass value;
		Node *left,*right,*up;
		int height;
		Node() : left(nullptr),right(nullptr),up(nullptr),height(1)
		{
		}
	};
public:
	class NodeHandle
	{
	friend BinaryTree <KeyClass,ValueClass>;
	private:
		Node *ptr;
	public:
		inline void Nullify(void)
		{
			ptr=nullptr;
		}
		inline bool IsNull(void) const
		{
			return ptr==nullptr;
		}
		inline bool IsNotNull(void) const
		{
			return ptr!=nullptr;
		}
		inline bool operator==(NodeHandle hd) const
		{
			return this->ptr==hd.ptr;
		}
		inline bool operator!=(NodeHandle hd) const
		{
			return this->ptr!=hd.ptr;
		}
		inline bool operator==(std::nullptr_t) const
		{
			return ptr==nullptr;
		}
		inline bool operator!=(std::nullptr_t) const
		{
			return ptr!=nullptr;
		}
	};
protected:
	Node *GetNode(NodeHandle ndHd)
	{
		if(ndHd.IsNotNull())
		{
			return ndHd.ptr;
		}
		return nullptr;
	}
	const Node *GetNode(NodeHandle ndHd) const
	{
		if(ndHd.IsNotNull())
		{
			return ndHd.ptr;
		}
		return nullptr;
	}
	static NodeHandle MakeHandle(Node *nodePtr)
	{
		NodeHandle ndHd;
		ndHd.ptr=nodePtr;
		return ndHd;
	}

	bool UpdateHeight(Node *nodePtr)
	{
			int leftHeight=1,rightHeight=1;
			if(nullptr!=nodePtr->left)
			{
					leftHeight=nodePtr->left->height+1;
			}
			if(nullptr!=nodePtr->right)
			{
					rightHeight=nodePtr->right->height+1;
			}
			int newHeight=1;
			if(leftHeight>rightHeight)
			{
					newHeight=leftHeight;
			}
			else
			{
					newHeight=rightHeight;
			}
			if(newHeight!=nodePtr->height)
			{
					nodePtr->height=newHeight;
					return true;
			}
			return false;
	}

	void UpdateHeightCascade(Node *nodePtr)
	{
			bool first=true;
			while(nullptr!=nodePtr)
			{
					auto changed=UpdateHeight(nodePtr);
					if(true!=first && true!=changed)
					{
							break;
					}
					nodePtr=nodePtr->up;
					first=false;
			}
	}

private:
	Node *root;
	long long int nElem;
	bool VineMode;

public:
	bool AutoRebalance;

public:
	BinaryTree()
	{
		root=nullptr;
		nElem=0;
		VineMode = false;
		AutoRebalance = false;
	}
	~BinaryTree()
	{
		CleanUp();
	}
	void CleanUp(void)
	{
		CleanUp(GetNode(RootNode()));
	}
private:
	void CleanUp(Node *nodePtr)
	{
		if(nullptr!=nodePtr)
		{
			CleanUp(nodePtr->left);
			CleanUp(nodePtr->right);
			delete nodePtr;
		}
	}
public:
	static NodeHandle Null(void)
	{
		NodeHandle ndHd;
		ndHd.ptr=nullptr;
		return ndHd;
	}
	NodeHandle RootNode(void) const
	{
		return MakeHandle(root);
	}
	NodeHandle Left(NodeHandle ndHd) const
	{
		auto nodePtr=GetNode(ndHd);
		if(nullptr!=nodePtr)
		{
			return MakeHandle(nodePtr->left);
		}
		return Null();
	}
	NodeHandle Up(NodeHandle ndHd) const
	{
		auto nodePtr=GetNode(ndHd);
		if(nullptr!=nodePtr)
		{
			return MakeHandle(nodePtr->up);
		}
		return Null();
	}
	NodeHandle Right(NodeHandle ndHd) const
	{
		auto nodePtr=GetNode(ndHd);
		if(nullptr!=nodePtr)
		{
			return MakeHandle(nodePtr->right);
		}
		return Null();
	}

	long long int GetN(void) const
	{
		return nElem;
	}
	const KeyClass &GetKey(NodeHandle ndHd) const
	{
		// This will crash if ndHd==nullptr.  Therefore, ndHd must be non-null to use this function.
		return GetNode(ndHd)->key;
	}
	ValueClass &GetValue(NodeHandle ndHd)
	{
		// This will crash if ndHd==nullptr.  Therefore, ndHd must be non-null to use this function.
		return GetNode(ndHd)->value;
	}
	const ValueClass &GetValue(NodeHandle ndHd) const
	{
		// This will crash if ndHd==nullptr.  Therefore, ndHd must be non-null to use this function.
		return GetNode(ndHd)->value;
	}
	NodeHandle FindNode(const KeyClass &key) const
	{
		auto ndHd=RootNode();
		while(nullptr!=ndHd)
		{
			if(key==GetKey(ndHd))
			{
				return ndHd;
			}
			if(key<GetKey(ndHd))
			{
				ndHd=Left(ndHd);
			}
			else
			{
				ndHd=Right(ndHd);
			}
		}
		return Null();
	}
	bool IsKeyIncluded(const KeyClass &key) const
	{
		return FindNode(key).IsNotNull();
	}
	int GetHeight(NodeHandle ndHd) const
	{
		auto nodePtr=GetNode(ndHd);
		if(nullptr!=nodePtr)
		{
			return nodePtr->height;
		}
		return 0;
	}

	NodeHandle Insert(const KeyClass &key,const ValueClass &value)
	{
		auto newNode=new Node;
		newNode->key=key;
		newNode->value=value;

		auto ndHd=RootNode();
		if(ndHd.IsNull())
		{
			root=newNode;
		}
		else
		{
			while(ndHd.IsNotNull())
			{
				if(key<GetKey(ndHd))
				{
					if(Left(ndHd)!=nullptr)
					{
						ndHd=Left(ndHd);
					}
					else
					{
						GetNode(ndHd)->left=newNode;
						newNode->up=GetNode(ndHd);
						break;
					}
				}
				else
				{
					if(Right(ndHd)!=nullptr)
					{
						ndHd=Right(ndHd);
					}
					else
					{
						GetNode(ndHd)->right=newNode;
						newNode->up=GetNode(ndHd);
						break;
					}
				}
			}
		}
		UpdateHeightCascade(newNode);
		nElem++;

		// Perform the rebalance, if necessary
		if(this->AutoRebalance)
			this->AVL_Rebalance(this->MakeHandle(newNode));

		return MakeHandle(newNode);
	}

	NodeHandle First(void) const
	{
		auto ndHd=RootNode();
		while(Left(ndHd).IsNotNull())
		{
			ndHd=Left(ndHd);
		}
		return ndHd;
	}
	NodeHandle FindNext(NodeHandle ndHd) const
	{
		auto rightHd=Right(ndHd);
		if(rightHd.IsNotNull())
		{
			// Has a right sub-tree.
			// The next node is the left-most of the right sub-tree.
			ndHd=Right(ndHd);
			while(Left(ndHd).IsNotNull())
			{
				ndHd=Left(ndHd);
			}
			return ndHd;
		}
		else
		{
			// Does not have a right sub-tree.
			// Go up until it goes up from the left.
			while(ndHd.IsNotNull())
			{
				auto upHd=Up(ndHd);
				if(upHd.IsNotNull() && ndHd==Left(upHd))
				{
					return upHd;
				}
				ndHd=upHd;
			}
			return Null();
		}
	}
	NodeHandle Last(void) const;  // Do it in the assignment.
	NodeHandle FindPrev(NodeHandle ndHd) const;  // Do it in the assignment.



private:
	NodeHandle RightMostOf(NodeHandle ndHd)
	{
		while(Right(ndHd).IsNotNull())
		{
			ndHd=Right(ndHd);
		}
		return ndHd;
	}
	bool SimpleDetach(NodeHandle ndHd)
	{
		if(ndHd.IsNotNull())
		{
			auto upHd=Up(ndHd);
			auto rightHd=Right(ndHd);
			auto leftHd=Left(ndHd);
			if(rightHd.IsNull() && leftHd.IsNull())
			{
				if(upHd.IsNull()) // ndHd is a root.
				{
					root=nullptr;
				}
				else
				{
					auto upPtr=GetNode(upHd);
					if(Left(upHd)==ndHd)
					{
						upPtr->left=nullptr;
					}
					else if(Right(upHd)==ndHd)
					{
						upPtr->right=nullptr;
					}
					else
					{
						fprintf(stderr,"Error! Internal Tree Data Structure is broken.\n");
						return false;
					}
				}
				UpdateHeightCascade(GetNode(upHd));

				// Perform the rebalance, if necessary
				if(this->AutoRebalance)
					this->AVL_Rebalance(upHd);

				return true;
			}
			else if(rightHd.IsNull())
			{
				if(upHd.IsNull())
				{
					root=GetNode(leftHd);
					root->up=nullptr;
					return true;
				}
				else
				{
					// Connect upHd and leftHd
					auto upPtr=GetNode(upHd);
					auto leftPtr=GetNode(leftHd);
					if(Left(upHd)==ndHd)
					{
						upPtr->left=leftPtr;
						leftPtr->up=upPtr;
						UpdateHeightCascade(GetNode(upHd));

						// Perform the rebalance, if necessary
						if(this->AutoRebalance)
							this->AVL_Rebalance(upHd);

						return true;
					}
					else if(Right(upHd)==ndHd)
					{
						upPtr->right=leftPtr;
						leftPtr->up=upPtr;
						UpdateHeightCascade(GetNode(upHd));

						// Perform the rebalance, if necessary
						if(this->AutoRebalance)
							this->AVL_Rebalance(upHd);

						return true;
					}
					else
					{
						fprintf(stderr,"Error! Internal Tree Data Structure is broken.\n");
						return false;
					}
				}
			}
			else if(leftHd.IsNull())
			{
				if(upHd.IsNull())
				{
					root=GetNode(rightHd);
					root->up=nullptr;
					return true;
				}
				else
				{
					// Connect upHd and rightHd
					auto upPtr=GetNode(upHd);
					auto rightPtr=GetNode(rightHd);
					if(Left(upHd)==ndHd)
					{
						upPtr->left=rightPtr;
						rightPtr->up=upPtr;
						UpdateHeightCascade(GetNode(upHd));

						// Perform the rebalance, if necessary
						if(this->AutoRebalance)
							this->AVL_Rebalance(upHd);

						return true;
					}
					else if(Right(upHd)==ndHd)
					{
						upPtr->right=rightPtr;
						rightPtr->up=upPtr;
						UpdateHeightCascade(GetNode(upHd));

						// Perform the rebalance, if necessary
						if(this->AutoRebalance)
							this->AVL_Rebalance(upHd);

						return true;
					}
					else
					{
						fprintf(stderr,"Error! Internal Tree Data Structure is broken.\n");
						return false;
					}
				}
			}
			else
			{
				return false;
			}
		}
		return false;
	}
public:
	bool Delete(NodeHandle ndHd)
	{
		if(true==SimpleDetach(ndHd))
		{
			delete GetNode(ndHd);
			--nElem;
			return true;
		}
		else if(ndHd.IsNotNull())
		{
			// Right most of left. Always Simple-Detachable.
			// Also, since SimpleDetach of itself has failed, it must have a left sub-tree.
			auto RMOL=RightMostOf(Left(ndHd));
			if(true==SimpleDetach(RMOL))
			{
				// Now, RMOL needs to take position of ndHd.
				auto RMOLptr=GetNode(RMOL);
				auto upPtr=GetNode(Up(ndHd));
				auto leftPtr=GetNode(Left(ndHd));
				auto rightPtr=GetNode(Right(ndHd));

				auto upOfRMOLptr=RMOLptr->up;
				if(upOfRMOLptr==GetNode(ndHd))
				{
					upOfRMOLptr=RMOLptr;	// Now it is correct.
				}

				if(nullptr==upPtr)
				{
					root=RMOLptr;
					root->up=nullptr;
				}
				else if(upPtr->left==GetNode(ndHd))
				{
					upPtr->left=RMOLptr;
					RMOLptr->up=upPtr;
				}
				else if(upPtr->right==GetNode(ndHd))
				{
					upPtr->right=RMOLptr;
					RMOLptr->up=upPtr;
				}
				else
				{
					fprintf(stderr,"Error! Internal Tree Data Structure is broken.\n");
					return false;
				}

				RMOLptr->left=leftPtr;
				if(nullptr!=leftPtr)
				{
					leftPtr->up=RMOLptr;
				}
				RMOLptr->right=rightPtr;
				if(nullptr!=rightPtr)
				{
					rightPtr->up=RMOLptr;
				}

				UpdateHeightCascade(upOfRMOLptr);

				// Perform the rebalance, if necessary
				if(this->AutoRebalance)
					this->AVL_Rebalance(this->MakeHandle(upOfRMOLptr));

				delete GetNode(ndHd);
				--nElem;
				return true;
			}
		}
		return false; // Cannot delete a null node.
	}

	bool RotateLeft(NodeHandle ndHd)
	{
		auto nodePtr=GetNode(ndHd);
		if(nullptr!=nodePtr && nullptr!=nodePtr->right)
		{
			auto rightPtr=nodePtr->right;
			auto leftOfRight=nodePtr->right->left;

			if(nullptr==nodePtr->up)
			{
				root=rightPtr;
				rightPtr->up=nullptr;
			}
			else
			{
				rightPtr->up=nodePtr->up;
				if(nodePtr->up->left==nodePtr)
				{
					nodePtr->up->left=rightPtr;
				}
				else
				{
					nodePtr->up->right=rightPtr;
				}
			}

			rightPtr->left=nodePtr;
			nodePtr->up=rightPtr;

			nodePtr->right=leftOfRight;
			if(nullptr!=leftOfRight)
			{
				leftOfRight->up=nodePtr;
			}

			UpdateHeight(nodePtr);
			UpdateHeightCascade(rightPtr);

			return true;
		}
		return false;
	}

	bool RotateRight(NodeHandle ndHd)
	{
		auto nodePtr=GetNode(ndHd);
		if(nullptr!=nodePtr && nullptr!=nodePtr->left)
		{
			auto leftPtr=nodePtr->left;
			auto rightOfLeft=nodePtr->left->right;

			if(nullptr==nodePtr->up)
			{
				root=leftPtr;
				leftPtr->up=nullptr;
			}
			else
			{
				leftPtr->up=nodePtr->up;
				if(nodePtr->up->left==nodePtr)
				{
					nodePtr->up->left=leftPtr;
				}
				else
				{
					nodePtr->up->right=leftPtr;
				}
			}

			leftPtr->right=nodePtr;
			nodePtr->up=leftPtr;

			nodePtr->left=rightOfLeft;
			if(nullptr!=rightOfLeft)
			{
				rightOfLeft->up=nodePtr;
			}

			UpdateHeight(nodePtr);
			UpdateHeightCascade(leftPtr);

			return true;
		}
		return false;
	}

	bool Tree2Vine(void)
	{
		if(this->root != nullptr)
		{
			// Make a pseudoroot
			Node pseudoroot;
			pseudoroot.right = this->root;
			this->root->up = &pseudoroot;

			// Start the process of Tree to Vine
			Node *vine_tail = &pseudoroot, *remainder = vine_tail->right;
			int size = 0;

			while(remainder != nullptr)
			{
				if(remainder->left == nullptr)
				{
					// Move down one level
					vine_tail = remainder;
					remainder = remainder->right;
					size += 1;
				}
				else
				{
					// RotateRight
					this->RotateRight(this->MakeHandle(remainder));
					remainder = remainder->up;
					vine_tail->right = remainder;
				}
			}
			this->VineMode = true;

			// Recover the root
			this->root = pseudoroot.right;
			this->root->up = nullptr;
			return true;
		}

		return false;
	}

private:
	bool Compression(Node *pseudoroot, int count)
	{
		Node *scanner = pseudoroot, *child = nullptr;

		// Start Compression
		for(int i = 0; i < count; i ++)
		{
			// Move into position
			child = scanner->right;
			scanner->right = child->right;
			child->right->up = scanner;
			scanner = scanner->right;

			// Modify the node
			child->right = scanner->left;
			if(scanner->left != nullptr)
				scanner->left->up = child;
			scanner->left = child;
			if(child != nullptr)
				child->up = scanner;
			else
				printf("[Compression]: Child should not be null.\n");
		}

		return true;
	}

public:
	bool Vine2Tree(void)
	{
		if(VineMode != false && this->root != nullptr)
		{
			this->VineMode = false;

			// Start the Vine to Tree Process
			Node pseudoroot;
			pseudoroot.right = this->root;
			this->root->up = &pseudoroot;

			int size = this->nElem;
			int leaf_count = size + 1 - pow(2, int(log2(size + 1)));

			this->Compression(&pseudoroot, leaf_count);
			size -= leaf_count;

			while(size > 1)
			{
				this->Compression(&pseudoroot, size / 2);
				size /= 2;
			}

			// Retrive the new root and
			// Thank you, pseudoroot!
			this->root = pseudoroot.right;
			this->root->up = nullptr;

			return true;
		}

		return false;
	}

private:
	int CheckBalance(Node *ndptr)
	{
		if(ndptr == nullptr)
			return 0;

		int left_height, right_height;
		ndptr->left != nullptr? left_height = ndptr->left->height : left_height = 0;
		ndptr->right != nullptr? right_height = ndptr->right->height : right_height = 0;

		return (left_height - right_height);
	}

protected:
	bool AVL_Rebalance(NodeHandle ndHd)
	{
		// Assert the validness of the input
		if(!ndHd.IsNotNull())
			return false;

		auto ndptr = GetNode(ndHd);
		int balance = this->CheckBalance(ndptr);
		Node *current = ndptr;

		while(current != nullptr)
		{
			if(-2 < balance && balance < 2)
			{
				// Proceed to the upstream node
				current = current->up;
				balance = this->CheckBalance(current);
				continue;
			}

			// OK, which mean a AVL rebalance is needed
			if(balance >= 2)
			{
				// Check the balance of the left-side node
				Node *left_node = current->left;
				if(left_node == nullptr)
				{
					printf("This left node is not supposed to be nullptr.\n");
					return false;
				}

				int left_balance = CheckBalance(left_node);
				if(left_balance >= 0)
				{
					// This is a LL case, do a right rotation on current
					this->RotateRight(this->MakeHandle(current));
				}
				else
				{
					// This is a LR case, turn it into LL and do the same
					this->RotateLeft(this->MakeHandle(left_node));
					this->RotateRight(this->MakeHandle(current));
				}
			}
			else
			{
				// Check the balance of the right-side node
				Node *right_node = current->right;
				if(right_node == nullptr)
				{
					printf("This right node is not supposed to be nullptr.\n");
					return false;
				}

				int right_balance = CheckBalance(right_node);
				if(right_balance >= 0)
				{
					// This is a RR case, do a left rotation on current
					this->RotateLeft(this->MakeHandle(current));
				}
				else
				{
					// This is a RL case, turn it into LL and do the same
					this->RotateRight(this->MakeHandle(right_node));
					this->RotateLeft(this->MakeHandle(current));
				}
			}

			// Proceed to the upstream node
			current = current->up;
			balance = this->CheckBalance(current);
		}

		return true;
	}
};
