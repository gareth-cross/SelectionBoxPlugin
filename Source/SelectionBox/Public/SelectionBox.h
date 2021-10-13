// Copyright 2021 Gareth Cross.
#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleManager.h"

class FSelectionBoxModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
